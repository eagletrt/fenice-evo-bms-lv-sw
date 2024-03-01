.PHONY: clean

build: test.c
	@gcc -I../Inc/ -I../Lib/micro-libs/Unity/src/ ../Lib/micro-libs/Unity/src/unity.c ../Src/measure_check.c test.c -o test.out

test: test.c
	@gcc -D UNITY_OUTPUT_COLOR=1 -I../Inc/ -I../Lib/micro-libs/Unity/src/ ../Lib/micro-libs/Unity/src/unity.c ../Src/measure_check.c test.c -o test.out 2>/dev/null
	@./test.out
	@$(MAKE) -s clean

clean:
	@rm test.out