/**
 * @file frq_test_bulk.cpp
 * @author Raymond Turrisi <rturrisi (at) mit (dot) edu>
 * @brief Running several frequency response tests as individual experiments
 * @version 0.1
 * @date 2022-11-28
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
    bool verbose = true; // whether or not you want to be updated during the test - not advised for high performance sampling
    float update_frq = 0.5; // frequency which you want to receive updates - the more printouts you receive per second the greater you comprimise performance

    float center_pressure = 50; //percent
    float amplitude = 8.33; //percent
    float duration = 20;
    int i = 1;

    float frq_lb = 0.25, frq_ub = 5;
    float frq_step = 0.25;
    for(float desired_frq_hz = frq_lb; desired_frq_hz <= frq_ub; desired_frq_hz+=frq_step) {
        printf("On test %d: %0.2f/%0.2f\n", i, desired_frq_hz, frq_ub);
        float frq_rads = 2*3.14*desired_frq_hz;
        char buffer[64];
        memset(buffer, '\0', 64);
        snprintf(buffer, 64, "frq_test_rads-%0.2f", frq_rads);
        char *dot = strchr(buffer,'.');
        if (dot)
        *dot = ',';
        dyn_test.run_frq_response(center_pressure, amplitude, frq_rads, duration, buffer, verbose, update_frq);
        i++;
    }
    
    return 0;
}