/**
 * @file simple_demo.cpp
 * @author Raymond Turrisi <rturrisi (at) mit (dot) edu>
 * @brief Demonstration for how to use the McKibben Commander class
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

/**
 * @brief Runs a demonstration test / serves as a boiler plate reference for 
 * working with the library
 * 
 * @param ac - not used
 * @param av - not used
 * @return int - success (0) or (n!=0) failure
 */

/* 
    This is required - if ctrl c is hit during runtime, then the destructor is not called, and the connection
    is left open, and the system will remain pressurized and the state that it was in, which may be
    consequential should you reboot the program (system can be left pressurized, and then on your next run you
    may start with disabling the brakes, which will cause a sharp impulse which can be hazardous to the system)
*/

static void *callback_data;
void sig_handler(int signum) {
    McKibbenCommander *commander_ref = (McKibbenCommander*)callback_data;
    printf("Callback %s\n", commander_ref->repr().c_str());
    commander_ref->~McKibbenCommander();
    exit(signum);
}


int main(int ac, char* av[]) {
    /*
        Boot up the system and instantiate connections
    */
    McKibbenCommander commander;
    if(commander.initSystem() == false) {
        exit(1);
    }
    /*
        Assign commander pointer to a general void pointer, and then register the signal 
        handler which uses this data to ensure a safe shutdown of the system with an 
        early termination
    */
    callback_data = (void*)&commander;
    signal(SIGINT, sig_handler); //ctrl-c
    signal(SIGABRT, sig_handler);
    signal(SIGKILL, sig_handler); //kill -9 pidnum
    signal(SIGQUIT, sig_handler);

    /*
        Opens the file as YY-MM-DD_hhmmss.dat
    */
    FILE *fptr;
    time_t rawtime;
    struct tm * timeinfo;
    char buffer [120];
    memset(buffer, '\0', 120);
    time (&rawtime);
    timeinfo = localtime (&rawtime);
    strftime (buffer,120,"data/%F_%H%M%S_data.dat",timeinfo);

    /*
        Starts local timers
    */
    std::chrono::time_point<std::chrono::high_resolution_clock> test_start, now;
    std::chrono::duration<double> duration;
    test_start = std::chrono::high_resolution_clock::now();

    /*
        Resets the time to reflect starting from 0 ms, updates the cache, and disables the brakes
    */
    commander.m_reset_time();
    commander.read_all();
    commander.disable_brakes();

    fptr = fopen(buffer, "a");
    fprintf(fptr, "%s\n", commander.header(",").c_str());
    
    /*
        Lower and upper bound millivolt outputs - this block was used to measure the fastest sample rate for 
        the system - on Mac Laptop with M1 chip it runs around 250 Hz

        If replicating this block, make sure you disable the brakes if it is your intent - it does not happen 
        automatically
    */
    int lb = 0, ub = 10000;
    for(int i = lb; i < ub; i++) {
        /*
            Reads at the fastest allowable rate
        */
        now = std::chrono::high_resolution_clock::now();
        duration = now - test_start; 
        commander.read_all();
        commander.set_pressure_output(i);
        fprintf(fptr, "%s\n", commander.repr().c_str());
        /*
            Print output occasionally to not seriously impact runtime performance for this demo, but also present progress incase 
            your computer is a little slow - should take about 30 seconds or so
        */
        if(i %100 == 0) {
            printf("%f: On - %d/%d\n", duration.count(), i,(ub-lb));
        }
    }

    commander.set_pressure_output(0);
    fclose(fptr);
    sleep(1);
    commander.enable_brakes();
    return 0;
}