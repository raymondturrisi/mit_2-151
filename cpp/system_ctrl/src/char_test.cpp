/**
 * @file char_test.cpp
 * @author Raymond Turrisi <rturrisi (at) mit (dot) edu>
 * @brief Demonstration of characterization test
 * @version 0.1
 * @date 2022-11-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "../lib/system_ctrl.hpp"
#include <stdio.h>
#include <unistd.h>
#include <fstream>
#include <chrono>
#include <time.h>

static void *callback_data;
void sig_handler(int signum) {
    McKibbenCommander *commander_ref = (McKibbenCommander*)callback_data;
    commander_ref->~McKibbenCommander();
    exit(signum);
}


int main(int ac, char* av[]) {
    /*
        Boot up the system and instantiate connections
    */
    McKibbenCommander *commander = new McKibbenCommander();
    if(commander->initSystem() == false) {
        exit(1);
    }
    /*
        IMPORTANT
        Assign commander pointer to a general void pointer, and then register the signal 
        handler which uses this data to ensure a safe shutdown of the system with an 
        early termination
        Alternatively, you will need to open brickv and connect in order to restore the system
    */
    callback_data = (void*)&commander;
    signal(SIGINT, sig_handler); //ctrl-c
    signal(SIGABRT, sig_handler);
    signal(SIGKILL, sig_handler); //kill -9 pidnum
    signal(SIGQUIT, sig_handler);
    commander->initSystem();

    //instantiate a dynamic test
    DynamicTest dyn_test(commander);
    //can modify default parameters - verbose is by default off, and the update frequency if you enable verbose
    dyn_test.run_characterization_test(0, 100, 10, "characterization", true, 1);
    return 0;
}