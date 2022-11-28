
/*
    TODO

    - Write program for connecting to all the devices and reading the data once
    - Write program for controlling the pressure regulators
    -
*/

#include <stdio.h>
#include <string>
#include <cstring>
#define IPCON_EXPOSE_MILLISLEEP
#include "ip_connection.h"
#include "brick_master.h"
#include "bricklet_industrial_analog_out_v2.h"
#include "bricklet_industrial_dual_0_20ma_v2.h"
#include "bricklet_industrial_dual_relay.h"
#include "bricklet_load_cell_v2.h"

#define HOST "localhost"
#define PORT 4223
using namespace std;

int main(int ac, char *av[])
{
    char UID_py1[] = "GZM"; // peizo output
    char UID_k1[] = "Kr4";  // relay output
    char UID_px[] = "Hfe";  // pressure sensors (supply & controlled)
    char UID_zx1[] = "Ji8"; // position sensor
    char UID_lc1[] = "S1C"; // load cell
    char UID_master1[] = "6EG9ps";
    char UID_master2[] = "6s6yLQ";

    IPConnection ipcon;
    ipcon_create(&ipcon);
    // Connect to brickd
    if (ipcon_connect(&ipcon, HOST, PORT) < 0)
    {
        fprintf(stderr, "Could not connect\n");
        return 1;
    }

    // MASTER BRICKLET
    
    // Create device object
    Master master1;
    master_create(&master1, UID_master1, &ipcon);
    Master master2;
    master_create(&master2, UID_master2, &ipcon);
    // Don't use device before ipcon is connected

    // Get current stack voltage
    uint16_t stack_voltage;
    if (master_get_stack_voltage(&master1, &stack_voltage) < 0)
    {
        fprintf(stderr, "Could not get stack voltage, probably timeout\n");
        return 1;
    }
    printf("Stack Voltage 1: %f V\n", stack_voltage / 1000.0);
    if (master_get_stack_voltage(&master2, &stack_voltage) < 0)
    {
        fprintf(stderr, "Could not get stack voltage, probably timeout\n");
        return 1;
    }

    printf("Stack Voltage 2: %f V\n", stack_voltage / 1000.0);

    // Get current stack current
    uint16_t stack_current;
    if (master_get_stack_current(&master1, &stack_current) < 0)
    {
        fprintf(stderr, "Could not get stack current, probably timeout\n");
        return 1;
    }
    printf("Stack Current 1: %f A\n", stack_current / 1000.0);
    if (master_get_stack_current(&master2, &stack_current) < 0)
    {
        fprintf(stderr, "Could not get stack current, probably timeout\n");
        return 1;
    }

    printf("Stack Current 2: %f A\n", stack_current / 1000.0);
    

    
    // PIEZO OUT
    {
        // Create device object
        IndustrialAnalogOutV2 iao;
        industrial_analog_out_v2_create(&iao, UID_py1, &ipcon);
        // Don't use device before ipcon is connected

        // Set output current to 4.5mA
        
        industrial_analog_out_v2_set_voltage(&iao, 3300);
        industrial_analog_out_v2_set_enabled(&iao, true);

        printf("Press key to exit\n");
        getchar();

        industrial_analog_out_v2_set_voltage(&iao, 0);
        industrial_analog_out_v2_set_enabled(&iao, false);
        industrial_analog_out_v2_destroy(&iao);
    }
    
    // DUAL ANALOG READ IN - Pressure sensors
    {
        // Create device object
        IndustrialDual020mAV2 pressure_sensors;
        industrial_dual_0_20ma_v2_create(&pressure_sensors, UID_px, &ipcon);
        IndustrialDual020mAV2 position_sensor;
        industrial_dual_0_20ma_v2_create(&position_sensor, UID_zx1, &ipcon);
        // Don't use device before ipcon is connected

        // Get current voltage from channel 0
        int32_t v_p1;
        int32_t v_p2;
        int32_t v_dist;
        int err;
        err = industrial_dual_0_20ma_v2_get_current(&pressure_sensors, 0, &v_p1);
        if (err < 0)
        {
            fprintf(stderr, "Could not get voltage from pressure sensor ch 0, probably timeout - %d\n", err);
            //return 1;
        }
        err = industrial_dual_0_20ma_v2_get_current(&pressure_sensors, 1, &v_p2);
        if (err < 0)
        {
            fprintf(stderr, "Could not get voltage from pressure sensor ch 1, probably timeout - %d\n", err);
            //return 1;
        }
        err = industrial_dual_0_20ma_v2_get_current(&position_sensor, 1, &v_dist);
        if (err < 0)
        {
            fprintf(stderr, "Could not get voltage from position sensor, probably timeout - %d\n", err);
            //return 1;
        }
        printf("Current (Pressure Channel 0): %f mA\n", v_p1 /1000000.0);
        printf("Current (Pressure Channel 1): %f mA\n", v_p2 /1000000.0);
        printf("Current (Distance Channel 0): %f mA\n", v_dist /1000000.0);
        industrial_dual_0_20ma_v2_destroy(&pressure_sensors);
        industrial_dual_0_20ma_v2_destroy(&position_sensor);
    }
    // DUAL RELAY
    {
        // Create device object
    IndustrialDualRelay idr;
    industrial_dual_relay_create(&idr, UID_k1, &ipcon);
    // Don't use device before ipcon is connected

    // Turn relays alternating on/off 10 times with 1 second delay
    int i;
    for(i = 0; i < 3; ++i) {
        //millisleep(2000);
        //industrial_dual_relay_set_value(&idr, true, false);
        //millisleep(2000);
        //industrial_dual_relay_set_value(&idr, true, false);
    }
    industrial_dual_relay_destroy(&idr);
    }
    
    //LOAD CELL
    {
        // Create device object
        LoadCellV2 lc;
        load_cell_v2_create(&lc, UID_lc1, &ipcon);

        // Don't use device before ipcon is connected

        // Get current weight
        int32_t weight;
        if(load_cell_v2_get_weight(&lc, &weight) < 0) {
            fprintf(stderr, "Could not get weight, probably timeout\n");
            return 1;
        }

        printf("Weight: %d g\n", weight);
        load_cell_v2_destroy(&lc);
    }

    master_destroy(&master1);
    master_destroy(&master2);
    ipcon_destroy(&ipcon); // Calls ipcon_disconnect internally
    return 0;
}