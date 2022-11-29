/**
 * @file step_test.cpp
 * @author Raymond Turrisi <rturrisi (at) mit (dot) edu>
 * @brief Demonstration of step unit tests
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
    Sequence sequence;
    sequence.push_back(0, 50); //start at 4000, after getting there provide a 10 second resting time for the system
    sequence.push_back(10, 40); //at 10 seconds, send a step signal to 12000
    sequence.push_back(15, 50);
    sequence.push_back(20, 0); //at time 20, the test will terminate, and send a signal of 0 - you should put zero here, but it will automatically deflate the actuator
    bool verbose = true; // whether or not you want to be updated during the test - not advised for high performance sampling
    float update_frq = 1; // frequency which you want to receive updates - the more printouts you receive per second the greater you comprimise performance
    dyn_test.run_step(sequence, -1, "step", verbose, update_frq);

    return 0;
}