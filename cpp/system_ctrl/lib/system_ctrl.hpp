/**
 * @file system_ctrl.hpp
 * @author Raymond Turrisi <rturrisi (at) mit (dot) edu>
 * @brief System controller for the McKibben Actuator Test setup originally designed by Peter G. Morice
 * 
 * Here we have a minimal suitable implementation to last the lifetime of the project. Here you will find: 
 * McKibben Commander: 
 *  - A class which maintains the communications with the experimental setup/tinkerforge bricks
 *  - Methods for simplifiying the control process
 * Dynamic Test:
 *  - A class for managing dynamic tests with common functions and utilities for:
 *  - step input (WIP)
 *  - ramp input (TOOD)
 *  - Frequency Response (TODO)
 *  - Initial condition response (TODO)
 * @version 0.1
 * @date 2022-11-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */

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
#include <vector>
#include <thread>
#include <sys/stat.h>
#include <sys/types.h>

#define HOST "localhost"
#define PORT 4223
using namespace std;


int dirExists(const char *path)
{
    struct stat info;

    if(stat( path, &info ) != 0)
        return 0;
    else if(info.st_mode & S_IFDIR)
        return 1;
    else
        return 0;
}

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
                //sets the configuration to sample at 80 Hz
                load_cell_v2_set_configuration(&m_load_cell,1,0);
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

        void raise_brakes() {
            this->disable_brakes();
            m_timenow = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> elapsed_milliseconds = m_timenow - m_timestart;
            float lb=6000.0, ub = 20000.0;
            int steps = 40;
            int duration = 3000; //ms
            int time_interval = duration/steps;
            float increments = (ub-lb)/steps;
            int wait_time = 1000;
            for(int i = lb; i < ub; i+=increments) {
                this->set_pressure_output(i);
                this_thread::sleep_for(chrono::milliseconds(time_interval));
            }
            this_thread::sleep_for(chrono::milliseconds(wait_time));
            this->enable_brakes();
            this_thread::sleep_for(chrono::milliseconds(100));
            this->set_pressure_output(0);
        }
};

//Data transfer object

class Sequence {
    public:
        vector<float> m_times; //times for an event to occur
        vector<float> m_magnitude; //pressures in millivolts (0 to 10000)
        uint32_t size;
        Sequence() {
            size=0;
        }
        void push_back(float second, float magnitude_mv) {
            if(size == 0) {
                if(abs(second) >= 0.02) {
                    fprintf(stderr, "Error: Sequence <push_back>: Must specify an initial condition at time = 0 seconds. Got [%f, %f]\n", second, magnitude_mv);
                    exit(1);
                }
            } 

            if(second > 100) {
                fprintf(stdout, "Warning: Test is exceeding over 100 seconds\n");
            }
            if(magnitude_mv < 0 || magnitude_mv > 20000) {
                fprintf(stderr, "Error: Sequence <push_back>: Magnitude in millivolts is out of bounds. Received (%f), bounds [0, 20000]\n", magnitude_mv);
                exit(1);
            }
            m_times.push_back(second);
            m_magnitude.push_back(magnitude_mv);
            size++;
        }

        void operator = (const Sequence &seq_to_copy ) { 
            m_times = seq_to_copy.m_times;
            m_magnitude = seq_to_copy.m_magnitude;
            size = seq_to_copy.size;
      }

      string repr() {
            string repr_result = "[";
            for (int i = 0; i < size; i++) {
                repr_result+="["+to_string(m_times[i])+","+to_string(m_magnitude[i])+"]";
            }
            repr_result+="]";
            return repr_result;
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
    public:
        McKibbenCommander *m_commander;
        bool m_explicit_sample_intervals;
        int m_dt;
        string m_latest_nickname;
        Sequence m_latest_sequence;
        string m_fname;

        DynamicTest(McKibbenCommander *commander) {
            m_commander = commander;
            m_explicit_sample_intervals = false;
            m_dt = -1;
            m_latest_nickname = "";
        };
        ~DynamicTest() {
            m_commander->~McKibbenCommander();
        }
        void m_run_step(Sequence step_sequence, float dt, string nickname) {
            /*
                TODO: 
                [ ] Simplify common actions into functions to be used for other dynamic tests
                [ ] Utilize the nickname feature
                [ ] Remove repeated/unused variables
            */
            m_latest_nickname = nickname;
            m_latest_sequence = step_sequence;
            //Check inputs
            if(step_sequence.size < 2) {
                fprintf(stderr, "Error: DynamicTest <m_run_step> - Sequence must contain at least two members\n");
            }
            if(dt < 0) {
                m_explicit_sample_intervals = false;
            } else {
                if(dt < 0.004 && dt >= 0) {
                    fprintf(stdout, "Warning: DynamicTest <m_run_step> - Sample time is less than fastest observed sample time of 0.004 seconds (4 ms/250 Hz). Received %f\n", dt);
                }
                m_explicit_sample_intervals = true;
                m_dt = dt*1000; //seconds to milliseconds
            }
        
            FILE *data_fptr;
            FILE *notes_fptr;
            time_t rawtime;
            struct tm * timeinfo;
            char dirname_buf [256];
            char data_fname_buf [256];
            char notes_fname_buf [256];
            char data_fname_full_buf [256];
            char notes_fname_full_buf [256];
            memset(data_fname_buf, '\0', 256);
            memset(notes_fname_buf, '\0', 256);
            memset(dirname_buf, '\0', 256);
            memset(data_fname_full_buf, '\0', 256);
            memset(notes_fname_full_buf, '\0', 256);
            time (&rawtime);
            timeinfo = localtime (&rawtime);
            strftime (dirname_buf,256,"data/%F_exps",timeinfo);
            if (!dirExists("data")) {
                if (mkdir("data", 0777) == -1)
                    fprintf(stderr, "Error: DynamicTest <m_run_step> - Cannot make directory data/- %s\n", strerror(errno));
            }
            if (!dirExists(dirname_buf)) {
                if (mkdir(dirname_buf, 0777) == -1)
                    fprintf(stderr, "Error: DynamicTest <m_run_step> - Cannot make directory data/*exp/- %s\n", strerror(errno));
            }

            strftime (data_fname_buf,256,"%F_%H%M%S_data.dat",timeinfo);
            strftime (notes_fname_buf,256,"%F_%H%M%S_notes.dat",timeinfo);
            snprintf(data_fname_full_buf, 256, "%s/%s", dirname_buf, data_fname_buf);
            snprintf(notes_fname_full_buf, 256, "%s/%s", dirname_buf, notes_fname_buf);

            //printf("%s\n%s\n%s\n", dirname_buf, data_fname_full_buf, notes_fname_full_buf);

            std::chrono::time_point<std::chrono::high_resolution_clock> test_start, now, next_sample;
            test_start = std::chrono::high_resolution_clock::now();
            now = std::chrono::high_resolution_clock::now();
            next_sample = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> duration = now - test_start;
            
            //Raise brakes
            float lag_time = 3;
            printf("- - RAISING BRAKES: %0.2f seconds to clear space - abort with Ctrl-C - - -\n", lag_time/1000);
            while((duration.count()) < lag_time) {
                printf("%0.2f s / %0.2f s\r", duration.count(), lag_time);
                now = std::chrono::high_resolution_clock::now();
                duration = now - test_start;
            }
            printf("\n");
            m_commander->raise_brakes();

            //Go to initial value and hold for 5 seconds

            test_start = std::chrono::high_resolution_clock::now();
            now = std::chrono::high_resolution_clock::now();
            duration = now - test_start;
            float rest_time = 5;
            
            int lb=0, ub = step_sequence.m_magnitude[0];
            int steps = 40;
            int action_span = rest_time*1000; //ms
            int time_interval = action_span/steps;
            float increments = (ub-lb)/steps;
            int wait_time = 1000;
            for(int i = lb; i < ub; i+=(int)increments) {
                now = std::chrono::high_resolution_clock::now();
                duration = now - test_start;
                m_commander->set_pressure_output(i);
                this_thread::sleep_for(chrono::milliseconds(time_interval));
                printf("Going to initial conditions: %0.2f s / %0.2f s\r", duration.count(), rest_time);
            }
            printf("\n");

            this_thread::sleep_for(chrono::milliseconds(wait_time));
            test_start = std::chrono::high_resolution_clock::now();
            now = std::chrono::high_resolution_clock::now();
            duration = now - test_start;
            double start_time = duration.count();
            printf("- - Starting Test - -\n");
            m_commander->m_reset_time();
            data_fptr = fopen(data_fname_full_buf, "a");
            fprintf(data_fptr, "%s\n", m_commander->header(",").c_str());
            fclose(data_fptr);
            for(int i = 0; i <= step_sequence.size; i++) {
                float t = 0;
                if(i!=step_sequence.size-1) {
                    t = step_sequence.m_times[i+1];
                }
                int mag = (int)step_sequence.m_magnitude[i];
                bool sig_out = false;
                while((duration.count()) < start_time+t) {
                    data_fptr = fopen(data_fname_full_buf, "a");
                    m_commander->read_all();
                    fprintf(data_fptr, "%s\n", m_commander->repr().c_str());
                    now = std::chrono::high_resolution_clock::now();
                    duration = now - test_start;
                    if(!sig_out) {
                        m_commander->set_pressure_output(mag);
                        sig_out = true;
                    }
                    fclose(data_fptr);
                    if(m_explicit_sample_intervals) {
                        this_thread::sleep_for(chrono::milliseconds(m_dt));
                    }
                }
                m_commander->set_pressure_output(0);
            }
            printf("- - TEST DONE - -\n");
        }
};