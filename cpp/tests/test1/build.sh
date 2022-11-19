#!/bin/bash

clang++ -I ../../tinkerforge_libc/source/ \
test.cpp ../../tinkerforge_libc/source/*.o \
-o test 
