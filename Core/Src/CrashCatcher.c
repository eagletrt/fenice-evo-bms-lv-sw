/* Copyright (C) 2022  Adam Green (https://github.com/adamgreen)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include <CrashCatcher.h>
#include "CrashCatcherPriv.h"
#include "string.h"
#include <stdbool.h>
#include "main.h"


/* Test harness will define this value on 64-bit machine to provide upper 32-bits of pointer addresses. */
CRASH_CATCHER_TEST_WRITEABLE uint64_t g_crashCatcherTestBaseAddress;

/* The unit tests can point the core to a fake location for the SCB->CPUID register. */
CRASH_CATCHER_TEST_WRITEABLE uint32_t* g_pCrashCatcherCpuId = (uint32_t*)0xE000ED00;

/* The unit tests can point the core to a fake location for the fault status registers. */
CRASH_CATCHER_TEST_WRITEABLE FaultStatusRegisters* g_pCrashCatcherFaultStatusRegisters = (FaultStatusRegisters*)0xE000ED28;

/* The unit tests can point the core to a fake location for the Coprocessor Access Control Register. */
CRASH_CATCHER_TEST_WRITEABLE uint32_t* g_pCrashCatcherCoprocessorAccessControlRegister = (uint32_t*)0xE000ED88;


/* Fault handler will switch MSP to use this area as the stack while CrashCatcher code is running.
   NOTE: If you change the size of this buffer, it also needs to be changed in the HardFault_Handler (in
         FaultHandler_arm*.S) when initializing the stack pointer. */
uint32_t g_crashCatcherStack[CRASH_CATCHER_STACK_WORD_COUNT];


typedef struct
{
    const CrashCatcherExceptionRegisters* pExceptionRegisters;
    CrashCatcherStackedRegisters*         pSP;
    uint32_t                              flags;
    CrashCatcherInfo                      info;
} Object;


static Object initStackPointers(const CrashCatcherExceptionRegisters* pExceptionRegisters);
static uint32_t getAddressOfExceptionStack(const CrashCatcherExceptionRegisters* pExceptionRegisters);
static void* uint32AddressToPointer(uint32_t address);
static void advanceStackPointerToValueBeforeException(Object* pObject);
static int areFloatingPointRegistersAutoStacked(const Object* pObject);
static void initFloatingPointFlag(Object* pObject);
static int areFloatingPointCoprocessorsEnabled(void);
static void initIsBKPT(Object* pObject);
static int isBKPT(uint16_t instruction);
static uint8_t getBKPTValue(uint16_t instruction);
static int isBadPC();
static void setStackSentinel(void);
static void dumpSignature(void);
static void dumpFlags(const Object* pObject);
static void dumpR0toR3(const Object* pObject);
static void dumpR4toR11(const Object* pObject);
static void dumpR12(const Object* pObject);
static void dumpSP(const Object* pObject);
static void dumpLR_PC_PSR(const Object* pObject);
static void dumpMSPandPSPandExceptionPSR(const Object* pObject);
static void dumpFloatingPointRegisters(const Object* pObject);
static void dumpMemoryRegions(const CrashCatcherMemoryRegion* pRegion);
static void checkStackSentinelForStackOverflow(void);
static int isARMv6MDevice(void);
static void dumpFaultStatusRegisters(void);
static void advanceProgramCounterPastHardcodedBreakpoint(const Object* pObject);

extern UART_HandleTypeDef huart1;

uint8_t* flashAddress = CRASH_DEBUG_STATE_DUMP_ADDR;


void CrashCatcher_Entry(const CrashCatcherExceptionRegisters* pExceptionRegisters)
{
    Object object = initStackPointers(pExceptionRegisters);
    advanceStackPointerToValueBeforeException(&object);
    initFloatingPointFlag(&object);
    initIsBKPT(&object);
    
    //unlock and erase flash
    static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t SECTORError;
    HAL_FLASH_Unlock();
    uint32_t StartSector = FLASH_SECTOR_7;
	EraseInitStruct.TypeErase     = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange  = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector        = StartSector;
    EraseInitStruct.NbSectors     = 1;
    HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);

    //set fault happened flag in flash
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)CRASH_DEBUG_FLAGS_ADDR, 0xFF ^ CRASH_DEBUG_FAULT_HAPPENED_MASK);

    // do
    // {
        setStackSentinel();
        CrashCatcher_DumpStart(&object.info);
        // HAL_UART_Transmit(&huart1, (const uint8_t*)"--Signature\r\n", 13, 10);
        dumpSignature();
        // HAL_UART_Transmit(&huart1, (const uint8_t*)"--Flags\r\n", 9, 10);
        dumpFlags(&object);
        // HAL_UART_Transmit(&huart1, (const uint8_t*)"--R0-R3\r\n", 9, 10);
        dumpR0toR3(&object);
        // HAL_UART_Transmit(&huart1, (const uint8_t*)"--R4-R11\r\n", 10, 10);
        dumpR4toR11(&object);
        // HAL_UART_Transmit(&huart1, (const uint8_t*)"--R12\r\n", 7, 10);
        dumpR12(&object);
        // HAL_UART_Transmit(&huart1, (const uint8_t*)"--SP\r\n", 6, 10);
        dumpSP(&object);
        // HAL_UART_Transmit(&huart1, (const uint8_t*)"--LR-PC-PSR\r\n", 13, 10);
        dumpLR_PC_PSR(&object);
        // HAL_UART_Transmit(&huart1, (const uint8_t*)"--MSP-PSP-ExceptionPSR\r\n", 24, 10);
        dumpMSPandPSPandExceptionPSR(&object);
        if (object.flags & CRASH_CATCHER_FLAGS_FLOATING_POINT) {
            HAL_UART_Transmit(&huart1, (uint8_t*)"--FloatingPoinRegisters\r\n", 25, 10);
            dumpFloatingPointRegisters(&object);
        }
        // HAL_UART_Transmit(&huart1, (const uint8_t*)"--MemoryRegions\r\n", 17, 10);
        dumpMemoryRegions(CrashCatcher_GetMemoryRegions());
        if (!isARMv6MDevice()) {
            // HAL_UART_Transmit(&huart1, (const uint8_t*)"--FaultStatusRegisters\r\n", 24, 10);
            dumpFaultStatusRegisters();
        }
        checkStackSentinelForStackOverflow();
    // }
    // while (CrashCatcher_DumpEnd() == CRASH_CATCHER_TRY_AGAIN);

    //set fault saved flag in flash
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)CRASH_DEBUG_FLAGS_ADDR, 0xFF ^ CRASH_DEBUG_FAULT_SAVED_MASK);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uint32_t)CRASH_DEBUG_STATE_DUMP_END_POINTER_ADDR, (uint32_t)flashAddress);

    HAL_FLASH_Lock();

    // char buf[50];
    // int len = snprintf(buf, 50, "end=%p\n\r", flashAddress);
    // HAL_UART_Transmit(&huart1, (const uint8_t*)buf, len, 10);
    // len = snprintf(buf, 50, "end=%p\n\r", *(uint8_t*)CRASH_DEBUG_STATE_DUMP_END_POINTER_ADDR);
    // HAL_UART_Transmit(&huart1, (const uint8_t*)buf, len, 10);

    // advanceProgramCounterPastHardcodedBreakpoint(&object);
}

static Object initStackPointers(const CrashCatcherExceptionRegisters* pExceptionRegisters)
{
    Object object;
    object.pExceptionRegisters = pExceptionRegisters;
    object.info.sp = getAddressOfExceptionStack(pExceptionRegisters);
    object.pSP = uint32AddressToPointer(object.info.sp);
    object.flags = 0;
    return object;
}

static uint32_t getAddressOfExceptionStack(const CrashCatcherExceptionRegisters* pExceptionRegisters)
{
    if (pExceptionRegisters->exceptionLR & LR_PSP)
        return pExceptionRegisters->psp;
    else
        return pExceptionRegisters->msp;
}

static void* uint32AddressToPointer(uint32_t address)
{
    if (sizeof(uint32_t*) == 8)
        return (void*)(unsigned long)((uint64_t)address | g_crashCatcherTestBaseAddress);
    else
        return (void*)(unsigned long)address;
}

static void advanceStackPointerToValueBeforeException(Object* pObject)
{
    /* Cortex-M processor always push 8 integer registers on the stack. */
    pObject->info.sp += 8 * sizeof(uint32_t);
    /* ARMv7-M processors can also push 16 single-precision floating point registers, FPSCR and a padding word. */
    if (areFloatingPointRegistersAutoStacked(pObject))
        pObject->info.sp += (16 + 1 + 1) * sizeof(uint32_t);
    /* Cortex-M processor may also have had to force 8-byte alignment before auto stacking registers. */
    pObject->info.sp |= (pObject->pSP->psr & PSR_STACK_ALIGN) ? 4 : 0;
}

static int areFloatingPointRegistersAutoStacked(const Object* pObject)
{
    return 0 == (pObject->pExceptionRegisters->exceptionLR & LR_FLOAT);
}

static void initFloatingPointFlag(Object* pObject)
{
    if (areFloatingPointCoprocessorsEnabled())
        pObject->flags |= CRASH_CATCHER_FLAGS_FLOATING_POINT;
}

static int areFloatingPointCoprocessorsEnabled(void)
{
    static const uint32_t coProcessor10and11EnabledBits = 5 << 20;
    uint32_t              coprocessorAccessControl = *g_pCrashCatcherCoprocessorAccessControlRegister;

    return (coprocessorAccessControl & (coProcessor10and11EnabledBits)) == coProcessor10and11EnabledBits;
}

static void initIsBKPT(Object* pObject)
{
    int wasBKPT = 0;
    uint8_t bkptNumber = 0;

    /* On ARMv7M, can use fault status registers to determine if bad PC was cause of fault before checking to see if
       it points to a BKPT instruction. */
    if (CRASH_CATCHER_ISBKPT_SUPPORT && (isARMv6MDevice() || !isBadPC()))
    {
        const uint16_t* pInstruction = uint32AddressToPointer(pObject->pSP->pc);
        uint16_t instruction = *pInstruction;
        wasBKPT = isBKPT(instruction);
        if (wasBKPT)
             bkptNumber = getBKPTValue(instruction);
    }
    pObject->info.isBKPT = wasBKPT;
    pObject->info.bkptNumber = bkptNumber;
}

static int isBKPT(uint16_t instruction)
{
    return (instruction & 0xFF00) == 0xBE00;
}

static uint8_t getBKPTValue(uint16_t instruction)
{
    return instruction & 0x00FF;
}

static int isBadPC()
{
    const uint32_t IACCVIOL = 1 << 0;
    const uint32_t IBUSERR = 1 << 8;
    const uint32_t badPCFaultBits = IACCVIOL | IBUSERR;

    return g_pCrashCatcherFaultStatusRegisters->CFSR & badPCFaultBits;
}

static void setStackSentinel(void)
{
    g_crashCatcherStack[0] = CRASH_CATCHER_STACK_SENTINEL;
}

static void dumpSignature(void)
{
    static const uint8_t signature[4] = {CRASH_CATCHER_SIGNATURE_BYTE0,
                                         CRASH_CATCHER_SIGNATURE_BYTE1,
                                         CRASH_CATCHER_VERSION_MAJOR,
                                         CRASH_CATCHER_VERSION_MINOR};

    CrashCatcher_DumpMemory(signature, CRASH_CATCHER_BYTE, sizeof(signature));
}

static void dumpFlags(const Object* pObject)
{
    CrashCatcher_DumpMemory(&pObject->flags, CRASH_CATCHER_BYTE, sizeof(pObject->flags));
}

static void dumpR0toR3(const Object* pObject)
{
    CrashCatcher_DumpMemory(&pObject->pSP->r0, CRASH_CATCHER_BYTE, 4 * sizeof(uint32_t));
}

static void dumpR4toR11(const Object* pObject)
{
    CrashCatcher_DumpMemory(&pObject->pExceptionRegisters->r4, CRASH_CATCHER_BYTE, (11 - 4 + 1) * sizeof(uint32_t));
}

static void dumpR12(const Object* pObject)
{
    CrashCatcher_DumpMemory(&pObject->pSP->r12, CRASH_CATCHER_BYTE, sizeof(uint32_t));
}

static void dumpSP(const Object* pObject)
{
    CrashCatcher_DumpMemory(&pObject->info.sp, CRASH_CATCHER_BYTE, sizeof(uint32_t));
}

static void dumpLR_PC_PSR(const Object* pObject)
{
    CrashCatcher_DumpMemory(&pObject->pSP->lr, CRASH_CATCHER_BYTE, 3 * sizeof(uint32_t));
}

static void dumpMSPandPSPandExceptionPSR(const Object* pObject)
{
    CrashCatcher_DumpMemory(&pObject->pExceptionRegisters->msp, CRASH_CATCHER_BYTE, 3 * sizeof(uint32_t));
}

static void dumpFloatingPointRegisters(const Object* pObject)
{
    uint32_t allFloatingPointRegisters[32 + 1];
    if (areFloatingPointRegistersAutoStacked(pObject))
    {
        /* Copy the upper floats first as that will cause a lazy copy of the auto-stacked registers. */
        CrashCatcher_CopyUpperFloatingPointRegisters(&allFloatingPointRegisters[16]);
    //     memcpy(&allFloatingPointRegisters[0], &pObject->pSP->floats, sizeof(pObject->pSP->floats));
    //     allFloatingPointRegisters[32] = pObject->pSP->fpscr;
    }
    else
    {
    //     CrashCatcher_CopyAllFloatingPointRegisters(allFloatingPointRegisters);
    }
    CrashCatcher_DumpMemory(allFloatingPointRegisters, CRASH_CATCHER_BYTE, sizeof(allFloatingPointRegisters));
}

static void dumpMemoryRegions(const CrashCatcherMemoryRegion* pRegion)
{
    while (pRegion && pRegion->startAddress != 0xFFFFFFFF)
    {
        /* Just dump the two addresses in pRegion.  The element size isn't required. */
        // HAL_UART_Transmit(&huart1, (const uint8_t*)"--MemoryRegion\r\n", 16, 10);
        CrashCatcher_DumpMemory(pRegion, CRASH_CATCHER_BYTE, 2 * sizeof(uint32_t));
        // HAL_UART_Transmit(&huart1, (const uint8_t*)"--MemoryContent\r\n", 17, 10);
        CrashCatcher_DumpMemory(uint32AddressToPointer(pRegion->startAddress),
                                pRegion->elementSize,
                                (pRegion->endAddress - pRegion->startAddress) / pRegion->elementSize);
        char buf[50];
        int len = snprintf(buf, 50, "dumped region %p\n\r", pRegion->startAddress);
        HAL_UART_Transmit(&huart1, (uint8_t*)buf, len, 10);
        pRegion++;
    }
}

static void checkStackSentinelForStackOverflow(void)
{
    if (g_crashCatcherStack[0] != CRASH_CATCHER_STACK_SENTINEL)
    {
        uint8_t value[4] = {0xAC, 0xCE, 0x55, 0xED};
        CrashCatcher_DumpMemory(value, CRASH_CATCHER_BYTE, sizeof(value));
    }
}

static int isARMv6MDevice(void)
{
    static const uint32_t armv6mArchitecture = 0xC << 16;
    uint32_t              cpuId = *g_pCrashCatcherCpuId;
    uint32_t              architecture = cpuId & (0xF << 16);

    return (architecture == armv6mArchitecture);
}

static void dumpFaultStatusRegisters(void)
{
    uint32_t                 faultStatusRegistersAddress = (uint32_t)(unsigned long)g_pCrashCatcherFaultStatusRegisters;
    CrashCatcherMemoryRegion faultStatusRegion[] = { {faultStatusRegistersAddress,
                                                      faultStatusRegistersAddress + sizeof(FaultStatusRegisters),
                                                      CRASH_CATCHER_BYTE},
                                                     {0xFFFFFFFF, 0xFFFFFFFF, CRASH_CATCHER_BYTE} };
    dumpMemoryRegions(faultStatusRegion);
}

static void advanceProgramCounterPastHardcodedBreakpoint(const Object* pObject)
{
    if (pObject->info.isBKPT)
        pObject->pSP->pc += 2;
}

static void dumpHexDigit(uint8_t nibble)
{
    static const char hexToASCII[] = "0123456789ABCDEF";

    // CrashCatcher_putc(hexToASCII[nibble]);
    HAL_UART_Transmit(&huart1, (uint8_t *)(hexToASCII + nibble), 1, 10);
}

static void dumpByteAsHex(uint8_t byte)
{
    dumpHexDigit(byte >> 4);
    dumpHexDigit(byte & 0xF);
}

void CrashCatcher_DumpMemory(const void* pvMemory, CrashCatcherElementSizes elementSize, size_t elementCount) {
    for (int i = 0 ; i < elementCount ; i++)
        {
            uint8_t data = *(uint8_t*)pvMemory;
            /* Only dump 16 bytes to a single line before introducing a line break. */
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)flashAddress++, data);
            // dumpByteAsHex(data);
            pvMemory++;
        }
}

CrashCatcherReturnCodes CrashCatcher_DumpEnd(void) {
    return CRASH_CATCHER_EXIT;
}

const CrashCatcherMemoryRegion* CrashCatcher_GetMemoryRegions(void) {
  static const CrashCatcherMemoryRegion regions[] = {
                {(uint32_t)CRASH_DEBUG_RAM_START_ADDR, (uint32_t)CRASH_DEBUG_RAM_START_ADDR + CRASH_DEBUG_RAM_LENGTH_KB * 1024, CRASH_CATCHER_BYTE},
                {0xFFFFFFFF, 0xFFFFFFFF, CRASH_CATCHER_BYTE}
                };
            return regions;

}

void CrashCatcher_DumpStart(const CrashCatcherInfo* pInfo) {

}

void crash_report_init() {
  uint8_t flash_status = *(uint8_t*)CRASH_DEBUG_FLAGS_ADDR;
  bool fault_happened = !(flash_status & CRASH_DEBUG_FAULT_HAPPENED_MASK);
  bool fault_saved = !(flash_status & CRASH_DEBUG_FAULT_SAVED_MASK);
  bool fault_sent = !(flash_status & CRASH_DEBUG_FAULT_SENT_MASK);


//   HAL_UART_Transmit(&huart1, (uint8_t*)"crash init\n\r", 12, 10);
  if (fault_happened) {
        HAL_UART_Transmit(&huart1, (uint8_t*)"fault happened\n\r", 16, 10);

        if (fault_sent) {
            HAL_UART_Transmit(&huart1, (uint8_t*)"fault sent\n\r", 12, 10);
        } else {
            HAL_UART_Transmit(&huart1, (uint8_t*)"fault not sent\n\r", 16, 10);
        }

        if (fault_saved) {
            HAL_UART_Transmit(&huart1, (uint8_t*)"fault saved\n\r", 13, 10);
            
            for (uint8_t* i = (uint8_t*)CRASH_DEBUG_STATE_DUMP_ADDR; (int)i < 0x807f484; ++i) {
                uint8_t data = *(uint8_t*)i;
                dumpByteAsHex(data);
            }
            
            //set fault sent flag in flash
            HAL_FLASH_Unlock();
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (uint32_t)CRASH_DEBUG_FLAGS_ADDR, 0xFF ^ CRASH_DEBUG_FAULT_SENT_MASK);
            HAL_FLASH_Lock();
        } else {
            HAL_UART_Transmit(&huart1, (uint8_t*)"fault not saved\n\r", 17, 10);
            //TODO: send syn with length -1
        }

    } else {
        HAL_UART_Transmit(&huart1, (uint8_t*)"fault not happened\n\r", 20, 10);
    }
}
