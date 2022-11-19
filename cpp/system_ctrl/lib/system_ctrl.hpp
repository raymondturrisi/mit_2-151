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

/**
 * @brief The McKibbenCommaner class wraps all the related peripherals for the experimental setup
 * so new tests can be conveniently written in very little code. Contains error checks and safety 
 * measures when working with the system
 * 
 */
class McKibbenCommander {
    public: //this would normally be private, but leaving it open incase people want to change the output format
        //Universal device id's
        //master bricks
        string m_uid_master_1;
        string m_uid_master_2;

        //pressure piezo output
        string m_uid_p_out;

        //brake relays
        string m_uid_brakes;

        //pressure sensor block (both sensors)
        string m_uid_pressure_sens;

        //position sensor block
        string m_uid_pos_sens;

        //load cell block
        string m_uid_load_cell;

        //ip connection object to connect to brickd
        IPConnection m_ipcon;

        //representative objects of bricks
        //master bricks
        Master m_master1;
        Master m_master2;

        //piezo output regulator
        IndustrialAnalogOutV2 m_piezo_out;

        //pressure sensors
        IndustrialDual020mAV2 m_pressure_sensors, m_position_sensor;

        //brake relays
        IndustrialDualRelay m_brake_relays;

        //load cell
        LoadCellV2 m_load_cell;

        //last set pressure
        float m_millivolt_out;

        //variables for immediately cacheing reads/sets
        //TODO: Update these types to "Reading"
        uint16_t m_m1_volts_dto, m_m2_volts_dto;
        float m_m1_volts, m_m2_volts;
        int32_t m_compressor_p_ma_dto, m_piezo_p_ma_dto, m_vertical_distance_ma_dto;
        float m_compressor_p_ma, m_piezo_p_ma, m_vertical_distance_ma;
        int32_t m_weight_grams_dto;
        float m_weight_grams;
        std::chrono::time_point<std::chrono::high_resolution_clock> m_timestart, m_timenow;
        bool m_brake_state;
        int last_err;
    public:
        /**
         * @brief Construct a new McKibben Commander object
         * 
         */
        McKibbenCommander() {
            m_uid_master_1 = "6EG9ps";
            m_uid_master_2 = "6s6yLQ";
            m_uid_p_out = "GZM";
            m_uid_brakes = "Kr4";
            m_uid_pressure_sens = "Hfe";
            m_uid_pos_sens = "Ji8";
            m_uid_load_cell = "S1C";
        };

        /**
         * @brief Initializes the system and checks to make sure we can connect to all the bricks and system prints the error if we cannot connect
         * and can be checked to prevent a segfault/successive test from executing
         * 
         * @return true If successful
         * @return false If unsuccessful
         * 
         * ERROR CODES 
         * E_OK = 0
         * E_TIMEOUT = -1
         * E_NO_STREAM_SOCKET = -2
         * E_HOSTNAME_INVALID = -3
         * E_NO_CONNECT = -4
         * E_NO_THREAD = -5
         * E_NOT_ADDED = -6 (unused since C/C++ bindings version 2.0.0)
         * E_ALREADY_CONNECTED = -7
         * E_NOT_CONNECTED = -8
         * E_INVALID_PARAMETER = -9
         * E_NOT_SUPPORTED = -10
         * E_UNKNOWN_ERROR_CODE = -11
         * E_STREAM_OUT_OF_SYNC = -12
         * E_INVALID_UID = -13
         * E_NON_ASCII_CHAR_IN_SECRET = -14
         * E_WRONG_DEVICE_TYPE = -15
         * E_DEVICE_REPLACED = -16
         * E_WRONG_RESPONSE_LENGTH = -17
         */
        bool initSystem() {
            /*
                Here we check each connection and cache the initial condition variable
                Throughout this program we have data_units_dto, and data_units. Data units is not to be accessed
                unless you want the raw data before a direct conversion - no processing is applied to the data, but 
                nanoamps and nanovolts are converted to millivolts for convenience. 
            */
            //connect to the brick daemon on your local machine
            ipcon_create(&m_ipcon);
            if (((last_err = ipcon_connect(&m_ipcon, HOST, PORT)) < 0))
            {
                fprintf(stderr, "ERR [%d]: Could not connect to brickd or system\n", last_err);
                return false;
            } else {
                //connect to master bricks
                master_create(&m_master1, m_uid_master_1.c_str(), &m_ipcon);
                master_create(&m_master2, m_uid_master_2.c_str(), &m_ipcon);
                
                if ((last_err = master_get_stack_voltage(&m_master1, &m_m1_volts_dto)) < 0)
                {
                    fprintf(stderr, "ERR [%d]: Could not connect to master brick 1\n", last_err);
                    return false;
                }
                m_m1_volts = (float)m_m1_volts_dto/1000.0;

                if ((last_err = master_get_stack_voltage(&m_master2, &m_m2_volts_dto)) < 0)
                {
                    fprintf(stderr, "ERR [%d]: Could not connect to master brick 2\n", last_err);
                    return false;
                }
                m_m2_volts=(float)m_m2_volts_dto/1000.0;

                //connect to brakes and enable
                industrial_dual_relay_create(&m_brake_relays, m_uid_brakes.c_str(), &m_ipcon);
                if((last_err = industrial_dual_relay_set_value(&m_brake_relays, false, true)) < 0) {
                    fprintf(stderr, "ERR [%d]: Could not connect to brakes\n", last_err);
                    return false;
                }
                m_brake_state = true;

                //connect to peripherals and enable all, while also checking to make sure we can communicate with them
                industrial_dual_0_20ma_v2_create(&m_pressure_sensors, m_uid_pressure_sens.c_str(), &m_ipcon);
                industrial_dual_0_20ma_v2_create(&m_position_sensor, m_uid_pos_sens.c_str(), &m_ipcon);

                if((last_err = industrial_dual_0_20ma_v2_get_current(&m_pressure_sensors, 1, &m_compressor_p_ma_dto)) < 0) {
                    fprintf(stderr, "ERR [%d]: Could not connect to pressure sensors\n", last_err);
                    return false;
                }
                m_compressor_p_ma=(float)m_compressor_p_ma_dto/1000000.0;
                if((last_err = industrial_dual_0_20ma_v2_get_current(&m_pressure_sensors, 0, &m_piezo_p_ma_dto)) < 0) {
                    fprintf(stderr, "ERR [%d]: Could not connect to pressure sensors\n", last_err);
                    return false;
                }
                m_piezo_p_ma=(float)m_piezo_p_ma_dto/1000000.0;
                if((last_err = industrial_dual_0_20ma_v2_get_current(&m_position_sensor, 1, &m_vertical_distance_ma_dto)) < 0) {
                    fprintf(stderr, "ERR [%d]: Could not connect to LVDI position sensor\n", last_err);
                    return false;
                }
                m_vertical_distance_ma=(float)m_vertical_distance_ma_dto/1000000.0;

                industrial_analog_out_v2_create(&m_piezo_out, m_uid_p_out.c_str(), &m_ipcon);
                if((last_err = industrial_analog_out_v2_set_enabled(&m_piezo_out, true)) < 0) {
                    fprintf(stderr, "ERR [%d]: Could not connect to piezo regulator\n", last_err);
                    return false;
                }
                m_piezo_p_ma = (float)m_piezo_p_ma_dto;
                

                load_cell_v2_create(&m_load_cell, m_uid_load_cell.c_str(), &m_ipcon);
                if((last_err = load_cell_v2_get_weight(&m_load_cell, &m_weight_grams_dto)) < 0) {
                    fprintf(stderr, "ERR [%d]: Could not connect to load cell\n", last_err);
                    return false;
                }
                m_weight_grams=(float)m_weight_grams_dto;
            }
            return true;
        }
        void m_reset_time() {
            /*
                Resets the timer from when the object was instantiated so data can be written starting from 0 milliseconds
            */
            m_timestart = std::chrono::high_resolution_clock::now();
            return;
        }

        ~McKibbenCommander() {
            /*
                Safely closes all connections and disables system output
            */

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

        /**
         * @brief Get the stack voltage read from master 1
         * 
         * @return int - millivolts
         */
        float get_stack_voltage_1() {
            /*
                Stack voltage from master brick 1 - should not be used independently - later there is a function
                which returns the average from the two bricks since the data can vary slightly
            */
            if ((last_err = master_get_stack_voltage(&m_master1, &m_m1_volts_dto)) < 0)
            {
                fprintf(stderr, "ERR [%d]: Could not connect to master brick 1\n", last_err);
                return -1;
            }
            m_m1_volts = (float)m_m1_volts_dto/1000.0;
            return m_m1_volts
;
        }
        /**
         * @brief Get the stack voltage read from master 2
         * 
         * @return int - millivolts
         */
        float get_stack_voltage_2() {
            /*
                Stack voltage from master brick 2 - should not be used independently - later there is a function
                which returns the average from the two bricks since the data can vary slightly
            */
            if ((last_err = master_get_stack_voltage(&m_master2, &m_m2_volts_dto)) < 0)
            {
                fprintf(stderr, "ERR [%d]: Could not connect to master brick 2\n", last_err);
                return -1;
            }
            m_m2_volts = (float)m_m2_volts_dto/1000.0;
            return m_m2_volts;
        }

        /**
         * @brief Returns the average voltage between the two master bricks
         * 
         * @return int - millivolts
         */
        float get_stack_voltage() {
            float v1 = get_stack_voltage_1();
            float v2 = get_stack_voltage_2();
            float mean = (v1+v2)/2;
            return mean;
        }
        /**
         * @brief Reads compressor pressure
         * 
         * @return int - millivolts
         */
        float read_compressor_pressure() {
            if((last_err = industrial_dual_0_20ma_v2_get_current(&m_pressure_sensors, 1, &m_compressor_p_ma_dto)) < 0) {
                fprintf(stderr, "ERR [%d]: Could not connect to pressure sensors\n", last_err);
                return -1;
            }
            m_compressor_p_ma=(float)m_compressor_p_ma_dto/1000000.0;
            return m_compressor_p_ma;
        }
        /**
         * @brief Reads the pressure after the piezo regulator
         * 
         * @return float - milliamps
         */
        float read_piezo_pressure() {
            if((last_err = industrial_dual_0_20ma_v2_get_current(&m_pressure_sensors, 0, &m_piezo_p_ma_dto)) < 0) {
                fprintf(stderr, "ERR [%d]: Could not connect to pressure sensors\n", last_err);
                return -1;
            }
            m_piezo_p_ma=(float)m_piezo_p_ma_dto/1000000.0;
            return m_piezo_p_ma;
        }
        /**
         * @brief Reads the position sensor
         * 
         * @return float - milliamps
         */
        float read_position_sensor() {
            if((last_err = industrial_dual_0_20ma_v2_get_current(&m_position_sensor, 1, &m_vertical_distance_ma_dto)) < 0) {
                fprintf(stderr, "ERR [%d]: Could not connect to LVDI position sensor\n", last_err);
                return -1;
            }
            m_vertical_distance_ma=(float)m_vertical_distance_ma_dto/1000000.0;
            return m_vertical_distance_ma;
        }

        /**
         * @brief Reads the load cell
         * 
         * @return float - grams
         */
        float read_load_cell() {
            if((last_err = industrial_dual_0_20ma_v2_get_current(&m_load_cell, 0, &m_weight_grams_dto)) < 0) {
                fprintf(stderr, "ERR [%d]: Could not connect to LVDI position sensor\n", last_err);
                return -1;
            }
            m_weight_grams=(float)m_weight_grams_dto;
            return m_weight_grams;
        }

        /**
         * @brief Enable the brakes, returning true if successful while updating the public 
         * brake state
         * 
         * @return true 
         * @return false 
         */
        bool enable_brakes() {
            if((last_err = industrial_dual_relay_set_value(&m_brake_relays, false, true)) < 0) {
                fprintf(stderr, "ERR [%d]: Could not enable brakes\n", last_err);
                return false;
            }
            m_brake_state = true;
            return true;
        }
        /**
         * @brief Disables the brakes, returning true if successful while updating the public 
         * brake state
         * 
         * @return true 
         * @return false 
         */
        bool disable_brakes() {
            if((last_err = industrial_dual_relay_set_value(&m_brake_relays, true, false)) < 0) {
                fprintf(stderr, "ERR [%d]: Could not disable brakes\n", last_err);
                return false;
            }
            m_brake_state = false;
            return true;
        }
        /**
         * @brief Set the pressure output in the form of millivolts
         * 
         * @param millivolt_out
         * @return true - if successful
         * @return false - if unsuccesful
         */
        bool set_pressure_output(int millivolt_out) {
            if((last_err = industrial_analog_out_v2_set_voltage(&m_piezo_out, millivolt_out)) < 0) {
                fprintf(stderr, "ERR [%d]: Could not set output pressure\n", last_err);
                return false;
            }
            m_millivolt_out = (float)millivolt_out;
            return true;
        }



        /**
         * @brief Reads all measurements and updates the cached values
         * The cached values can be accessed directly, or you can call
         * commander.repr() to get the associated string representation. 
         * 
         * While this does not return anything, you can access the updates states directly
         * as commander.m_compressor_p_ma and so on. 
         */
        void read_all(){
            get_stack_voltage();
            read_compressor_pressure();
            read_piezo_pressure();
            read_position_sensor();
            read_load_cell();
            return;
        }
        
        
        /**
         * @brief Prints the header/format in which the representation is constructed. 
         * Should use "," as a delimeter
         * 
         * @param delimeter 
         * @return string 
         */
        string header(string delimeter) {
            char buffer[128];
            m_timenow = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed_milliseconds = m_timenow - m_timestart;
            memset(buffer, '\0', 128);
            snprintf(buffer, 128, "T_(ms)%sSys_Volts_(V)%sComp_P_(mA)%sPiezo_P_(mA)%sPiezo_out_(mV)%sV_dist_(mA)%sLC_(grams)\n", 
                delimeter.c_str(),
                delimeter.c_str(),
                delimeter.c_str(),
                delimeter.c_str(),
                delimeter.c_str(),
                delimeter.c_str());
            return string(buffer);
        }
        /**
         * @brief Print the string representation with a specified delimeter. Should not be used, but left
         * as an option
         * 
         * @param delimiter 
         * @return string 
         */
        string repr(string delimiter) {
            char buffer[128];
            m_timenow = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed_milliseconds = m_timenow - m_timestart;
            memset(buffer, '\0', 128);
            snprintf(buffer, 128, "%0.3f%s%0.3f%s%0.3f%s%0.3f%s%0.0f%s%0.3f%s%0.0f", (float)elapsed_milliseconds.count(), delimiter.c_str(), 
                (m_m1_volts+m_m2_volts)/2.0, delimiter.c_str(), 
                m_compressor_p_ma, delimiter.c_str(), 
                m_piezo_p_ma, delimiter.c_str(),
                m_millivolt_out, delimiter.c_str(), 
                m_vertical_distance_ma, delimiter.c_str(), 
                m_weight_grams);
            return string(buffer);
        }
        /**
         * @brief Return the string representation of the latest data
         * 
         * @return string - formatted in the order which can be retrieved by the header() method
         */
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