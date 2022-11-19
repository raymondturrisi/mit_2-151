/**
 * @file rt_testing.cpp
 * @author Raymond Turrisi <rturrisi (at) mit (dot) edu>
 * @brief A real time testing play program - not for serious use - only development
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
    printf("Callback %s\n", commander_ref->repr().c_str());
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
        Assign commander pointer to a general void pointer, and then register the signal 
        handler which uses this data to ensure a safe shutdown of the system with an 
        early termination
    */
    callback_data = (void*)&commander;
    signal(SIGINT, sig_handler); //ctrl-c
    signal(SIGABRT, sig_handler);
    signal(SIGKILL, sig_handler); //kill -9 pidnum
    signal(SIGQUIT, sig_handler);
    commander->initSystem();

    DynamicTest dyn_test(commander);

    Sequence sequence;
    sequence.push_back(0, 4000);
    sequence.push_back(10, 12000);
    sequence.push_back(15, 4000);
    sequence.push_back(20, 0);
    dyn_test.m_run_step(sequence, -1, "nickname");
    return 0;
}