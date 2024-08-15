#! /bin/bash

openocd -f ./openocd.cfg -c "program build/fenice-bms-lv.elf verify reset exit"
