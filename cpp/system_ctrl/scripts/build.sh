#!/bin/bash

#Execute from: system_ctrl/
# $ ./scripts/build.sh
clang++ -O3 -I ../tinkerforge_libc/source/ -I lib/ \
src/simple_demo.cpp ../tinkerforge_libc/source/*.o \
-o ../../bin/simple_demo.exe 

clang++ -O3 -I ../tinkerforge_libc/source/ -I lib/ \
src/step_test.cpp ../tinkerforge_libc/source/*.o \
-o ../../bin/step_test.exe 

clang++ -O3 -I ../tinkerforge_libc/source/ -I lib/ \
src/ramp_test.cpp ../tinkerforge_libc/source/*.o \
-o ../../bin/ramp_test.exe 

clang++ -O3 -I ../tinkerforge_libc/source/ -I lib/ \
src/frq_test.cpp ../tinkerforge_libc/source/*.o \
-o ../../bin/frq_test.exe 

clang++ -O3 -I ../tinkerforge_libc/source/ -I lib/ \
src/frq_test_bulk.cpp ../tinkerforge_libc/source/*.o \
-o ../../bin/frq_test_bulk.exe 

clang++ -O3 -I ../tinkerforge_libc/source/ -I lib/ \
src/char_test.cpp ../tinkerforge_libc/source/*.o \
-o ../../bin/char_test.exe 
