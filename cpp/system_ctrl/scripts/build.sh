#!/bin/bash
clang++ -I ../tinkerforge_libc/source/ -I lib/ \
src/simple_demo.cpp ../tinkerforge_libc/source/*.o \
-o bin/simple_demo.exe 
