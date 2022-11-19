#include "ip_connection.h"
#include "brick_master.h"
#include "bricklet_industrial_analog_out_v2.h"
#include "bricklet_industrial_dual_0_20ma_v2.h"
#include "bricklet_industrial_dual_relay.h"
#include "bricklet_load_cell_v2.h"
#include <stdio.h>
#include <string>
#include <cstring>
#include <chrono>

#define HOST "localhost"
#define PORT 4223
using namespace std;

template <typename T> class Reading {
    //TODO: Change types to reading class and add time stamps at the time something is pulled
    public:
    T dat;
    double timestamp;
    string repr() {
        char buffer[64];
        memset(buffer, '\0', 64);
        snprintf(buffer, "%0.3f,%0.3f", static_cast<float>(dat), timestamp);
        return string(buffer);
    }
};

class McKibbenCommander {
    private:
        //Universal device id's
        //master bricks
        string m_uid_master_1;
        string m_uid_master_2;

        //pressure piezo output
        string m_uid_p_out;

        //brake relays
        string m_uid_brakes;

        //pressure sensor block (both sensors)
        string m_uid_p_sens;

        //position sensor block
        string m_uid_pos_sens;

        //load cell block
        string m_uid_load_cell;

        //ip connection object to connect to brickd
        IPConnection m_ipcon;

        //representative objects of bricks
        //master bricks
        Master m_master1, m_master2;

        //piezo output regulator
        IndustrialAnalogOutV2 m_piezo_out;

        //pressure sensors
        IndustrialDual020mAV2 m_pressure_sensors, m_position_sensor;

        //brake relays
        IndustrialDualRelay m_brake_relays;

        //load cell
        LoadCellV2 m_load_cell;

        //last set pressure
        int32_t m_milliamp_out;

        //variables for immediately cacheing reads/sets
        //TODO: Update these types to "Reading"
        uint16_t m_m1_volts, m_m2_volts;
        int32_t m_compressor_p_mv, m_piezo_p_mv, m_vertical_distance_mv;
        int32_t m_weight_grams;

        std::chrono::time_point<std::chrono::high_resolution_clock> m_timestart, m_timenow;

        int last_err;
    public:
        //constructor - must call initSystem and check return to boot up system
        McKibbenCommander() {
            m_uid_master_1 = "6EG9ps";
            m_uid_master_2 = "6s6yLQ";
            m_uid_p_out = "GZM";
            m_uid_brakes = "Kr4";
            m_uid_p_sens = "Hfe";
            m_uid_pos_sens = "Ji8";
            m_uid_load_cell = "S1C";
        };

        //boots up system - returns true if successful, otherwise prints an error code to output which can be found online, and returns false so your program can exit early
        bool initSystem() {
            //connect to the brick daemon on your local machine
            ipcon_create(&m_ipcon);
            if (((last_err = ipcon_connect(&m_ipcon, HOST, PORT)) < 0))
            {
                fprintf(stderr, "ERR [%d]: Could not connect to brickd or system\n", last_err);
                return false;
            } else {
                //connect to master bricks
                master_create(&m_master1, m_uid_master_1.c_str(), &m_ipcon);
                master_create(&m_master1, m_uid_master_2.c_str(), &m_ipcon);
                
                if ((last_err = master_get_stack_voltage(&m_master1, &m_m1_volts)) < 0)
                {
                    fprintf(stderr, "ERR [%d]: Could not connect to master brick 1\n", last_err);
                    return false;
                }

                if ((last_err = master_get_stack_voltage(&m_master2, &m_m2_volts)) < 0)
                {
                    fprintf(stderr, "ERR [%d]: Could not connect to master brick 2\n", last_err);
                    return false;
                }

                //connect to brakes and enable
                industrial_dual_relay_create(&m_brake_relays, m_uid_brakes.c_str(), &m_ipcon);

                if((last_err = industrial_dual_relay_set_value(&m_brake_relays, false, true)) < 0) {
                    fprintf(stderr, "ERR [%d]: Could not connect to brakes\n", last_err);
                    return false;
                }

                //connect to peripherals and enable all, while also checking to make sure we can communicate with them
                industrial_dual_0_20ma_v2_create(&m_pressure_sensors, m_uid_p_sens.c_str(), &m_ipcon);
                industrial_dual_0_20ma_v2_create(&m_position_sensor, m_uid_pos_sens.c_str(), &m_ipcon);

                if((last_err = industrial_dual_0_20ma_v2_get_current(&m_pressure_sensors, 0, &m_compressor_p_mv)) < 0) {
                    fprintf(stderr, "ERR [%d]: Could not connect to pressure sensors\n", last_err);
                    return false;
                }
                if((last_err = industrial_dual_0_20ma_v2_get_current(&m_position_sensor, 0, &m_vertical_distance_mv)) < 0) {
                    fprintf(stderr, "ERR [%d]: Could not connect to LVDI position sensor\n", last_err);
                    return false;
                }

                industrial_analog_out_v2_create(&m_piezo_out, m_uid_pos_sens.c_str(), &m_ipcon);
                if((last_err = industrial_analog_out_v2_set_enabled(&m_piezo_out, true)) < 0) {
                    fprintf(stderr, "ERR [%d]: Could not connect to piezo regulator\n", last_err);
                    return false;
                }

                load_cell_v2_create(&m_load_cell, m_uid_load_cell.c_str(), &m_ipcon);
                if((last_err = load_cell_v2_get_weight(&m_load_cell, &m_weight_grams)) < 0) {
                    fprintf(stderr, "ERR [%d]: Could not connect to load cell\n", last_err);
                    return false;
                }
            }
            return true;
        }
        void m_reset_time() {
            m_timestart = std::chrono::high_resolution_clock::now();
            return;
        }

        ~McKibbenCommander() {
            //closes all connections as appropriate

            //Brakes
            //enable brakes then close them down
            industrial_dual_relay_set_value(&m_brake_relays, false, true);
            industrial_dual_relay_destroy(&m_brake_relays);

            //Presure sensors
            //disable sensors
            industrial_dual_0_20ma_v2_destroy(&m_pressure_sensors);
            industrial_dual_0_20ma_v2_destroy(&m_position_sensor);

            //Pressure output
            //set piezo value to zero, distable output, then destroy the obj
            industrial_analog_out_v2_set_voltage(&m_piezo_out, 0);
            industrial_analog_out_v2_set_enabled(&m_piezo_out, false);
            industrial_analog_out_v2_destroy(&m_piezo_out);

            //Load cell
            load_cell_v2_destroy(&m_load_cell);

            //Shut down masters
            master_destroy(&m_master1);
            master_destroy(&m_master2);

            //Disconnect from brickd
            ipcon_destroy(&m_ipcon);
        };

        int get_stack_voltage_1() {
            if ((last_err = master_get_stack_voltage(&m_master1, &m_m1_volts)) < 0)
            {
                fprintf(stderr, "ERR [%d]: Could not connect to master brick 1\n", last_err);
                return -1;
            }
            return m_m1_volts;
        }
        int get_stack_voltage_2() {
            if ((last_err = master_get_stack_voltage(&m_master2, &m_m2_volts)) < 0)
            {
                fprintf(stderr, "ERR [%d]: Could not connect to master brick 2\n", last_err);
                return -1;
            }
            return m_m2_volts;
        }
        int get_stack_voltage() {
            int v1 = get_stack_voltage_1();
            int v2 = get_stack_voltage_2();
            int mean = (v1+v2)/2;
            return mean;
        }
        int read_compressor_pressure() {
            if((last_err = industrial_dual_0_20ma_v2_get_current(&m_pressure_sensors, 0, &m_compressor_p_mv)) < 0) {
                fprintf(stderr, "ERR [%d]: Could not connect to pressure sensors\n", last_err);
                return -1;
            }
            return m_compressor_p_mv;
        }
        int read_piezo_pressure() {
            if((last_err = industrial_dual_0_20ma_v2_get_current(&m_pressure_sensors, 1, &m_piezo_p_mv)) < 0) {
                fprintf(stderr, "ERR [%d]: Could not connect to pressure sensors\n", last_err);
                return -1;
            }
            return m_piezo_p_mv;
        }
        int read_position_sensor() {
            if((last_err = industrial_dual_0_20ma_v2_get_current(&m_position_sensor, 0, &m_vertical_distance_mv)) < 0) {
                fprintf(stderr, "ERR [%d]: Could not connect to LVDI position sensor\n", last_err);
                return -1;
            }
            return m_vertical_distance_mv;
        }
        bool enable_breaks() {
            if((last_err = industrial_dual_relay_set_value(&m_brake_relays, false, true)) < 0) {
                fprintf(stderr, "ERR [%d]: Could not enable brakes\n", last_err);
                return false;
            }
            return true;
        }
        int disable_breaks() {
            if((last_err = industrial_dual_relay_set_value(&m_brake_relays, false, true)) < 0) {
                fprintf(stderr, "ERR [%d]: Could not disable brakes\n", last_err);
                return false;
            }
            return true;
        }
        int set_pressure_output(int milliamp_out) {
            if((last_err = industrial_analog_out_v2_set_voltage(&m_piezo_out, milliamp_out)) < 0) {
                fprintf(stderr, "ERR [%d]: Could not set output pressure\n", last_err);
                return false;
            }
            m_milliamp_out = milliamp_out;
            return true;
        }
        
        string repr(string delimiter) {
            //TODO: Finish representation of data here
            char buffer[64];
            m_timenow = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed_milliseconds = m_timenow - m_timestart;
            memset(buffer, '\0', 64);
            snprintf(buffer, 64, "%0.3f%s\n", elapsed_milliseconds.count(), delimiter);
            return string(buffer);
        }

        string repr() {
            return repr(",");
        }

};

class DynamicTest {
    /*
        TODO: Implement a dynamic test class which runs after parameters are provided
        Some parameters to consider: 
         [ ] Pass a vector of "sequences" - a sequence is solely a timestamp to hold a signal and the signal to be output
            with this, you can pack a vector as little or as much as you would like [[10, 4000],[20, 8000]], with this, a signal is held until the next time step from 
            initialization is reached
        [ ] Compose utility functions for developing sequences for ramp, step, and sinusoidal patterns
     */
    private:

    public:

};