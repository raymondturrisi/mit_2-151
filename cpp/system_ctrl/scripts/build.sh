#!/bin/bash
clang++ -O3 -I ../tinkerforge_libc/source/ -I lib/ \
src/simple_demo.cpp ../tinkerforge_libc/source/*.o \
-o bin/simple_demo.exe 


clang++ -O3 -I ../tinkerforge_libc/source/ -I lib/ \
src/step_test.cpp ../tinkerforge_libc/source/*.o \
-o bin/step_test.exe 

clang++ -O3 -I ../tinkerforge_libc/source/ -I lib/ \
src/ramp_test.cpp ../tinkerforge_libc/source/*.o \
-o bin/ramp_test.exe 
