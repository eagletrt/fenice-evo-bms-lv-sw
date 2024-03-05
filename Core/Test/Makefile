.PHONY: clean

#Includes
INC_DIR=../Inc/
UNITY_DIR=../Lib/micro-libs/Unity/src/

#Soruce files
UNITY_C=../Lib/micro-libs/Unity/src/unity.c

MEASURE_CHECK_C=../Src/measure_check.c
MEASURE_CHECK_TEST_C=measure_check_test.c

#Commands
BUILD_COMMAND=gcc -Wall -I$(UNITY_DIR) -I$(INC_DIR) $(UNITY_C) $(MEASURE_CHECK_C) $(MEASURE_CHECK_TEST_C) -o test.out

build:
	$(BUILD_COMMAND)

test:
	@ $(BUILD_COMMAND) -D UNITY_OUTPUT_COLOR=1 2>/dev/null
	@./test.out
	@$(MAKE) -s clean

clean:
	@rm test.out