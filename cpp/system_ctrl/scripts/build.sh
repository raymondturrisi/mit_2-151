#!/bin/bash
clang++ -I ../tinkerforge_libc/source/ -I lib/ \
src/main.cpp ../tinkerforge_libc/source/*.o \
-o bin/main.exe 
