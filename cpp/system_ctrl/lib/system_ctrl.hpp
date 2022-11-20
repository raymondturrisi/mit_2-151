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
#include <cmath>
#define HOST "localhost"
#define PORT 4223
using namespace std;

/**
 * @brief Check to see if a directory exists
 *  taken from stackoverflow
 * @param path
 * @return int
 */
int dirExists(const char *path)
{
    struct stat info;

    if (stat(path, &info) != 0)
        return 0;
    else if (info.st_mode & S_IFDIR)
        return 1;
    else
        return 0;
}

/**
 * @brief The McKibbenCommaner class wraps all the related peripherals for the experimental setup
 * so new tests can be conveniently written in very little code. Contains error checks and safety
 * measures when working with the system
 *
 */
class McKibbenCommander
{
public: // this would normally be private, but leaving it open incase people want to change the output format
    // Universal device id's
    // master bricks
    string m_uid_master_1;
    string m_uid_master_2;

    // pressure piezo output
    string m_uid_p_out;

    // brake relays
    string m_uid_brakes;

    // pressure sensor block (both sensors)
    string m_uid_pressure_sens;

    // position sensor block
    string m_uid_pos_sens;

    // load cell block
    string m_uid_load_cell;

    // ip connection object to connect to brickd
    IPConnection m_ipcon;

    // representative objects of bricks
    // master bricks
    Master m_master1;
    Master m_master2;

    // piezo output regulator
    IndustrialAnalogOutV2 m_piezo_out;

    // pressure sensors
    IndustrialDual020mAV2 m_pressure_sensors, m_position_sensor;

    // brake relays
    IndustrialDualRelay m_brake_relays;

    // load cell
    LoadCellV2 m_load_cell;

    // last set pressure
    float m_millivolt_out;

    // variables for immediately cacheing reads/sets
    // TODO: Update these types to "Reading"
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
    McKibbenCommander()
    {
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
    bool initSystem()
    {
        /*
            Here we check each connection and cache the initial condition variable
            Throughout this program we have data_units_dto, and data_units. Data units is not to be accessed
            unless you want the raw data before a direct conversion - no processing is applied to the data, but
            nanoamps and nanovolts are converted to millivolts for convenience.
        */
        // connect to the brick daemon on your local machine
        ipcon_create(&m_ipcon);
        if (((last_err = ipcon_connect(&m_ipcon, HOST, PORT)) < 0))
        {
            fprintf(stderr, "ERR [%d]: Could not connect to brickd or system\n", last_err);
            return false;
        }
        else
        {
            // connect to master bricks
            master_create(&m_master1, m_uid_master_1.c_str(), &m_ipcon);
            master_create(&m_master2, m_uid_master_2.c_str(), &m_ipcon);

            if ((last_err = master_get_stack_voltage(&m_master1, &m_m1_volts_dto)) < 0)
            {
                fprintf(stderr, "ERR [%d]: Could not connect to master brick 1\n", last_err);
                return false;
            }
            m_m1_volts = (float)m_m1_volts_dto / 1000.0;

            if ((last_err = master_get_stack_voltage(&m_master2, &m_m2_volts_dto)) < 0)
            {
                fprintf(stderr, "ERR [%d]: Could not connect to master brick 2\n", last_err);
                return false;
            }
            m_m2_volts = (float)m_m2_volts_dto / 1000.0;

            // connect to brakes and enable
            industrial_dual_relay_create(&m_brake_relays, m_uid_brakes.c_str(), &m_ipcon);
            if ((last_err = industrial_dual_relay_set_value(&m_brake_relays, false, true)) < 0)
            {
                fprintf(stderr, "ERR [%d]: Could not connect to brakes\n", last_err);
                return false;
            }
            m_brake_state = true;

            // connect to peripherals and enable all, while also checking to make sure we can communicate with them
            industrial_dual_0_20ma_v2_create(&m_pressure_sensors, m_uid_pressure_sens.c_str(), &m_ipcon);
            industrial_dual_0_20ma_v2_create(&m_position_sensor, m_uid_pos_sens.c_str(), &m_ipcon);

            if ((last_err = industrial_dual_0_20ma_v2_get_current(&m_pressure_sensors, 1, &m_compressor_p_ma_dto)) < 0)
            {
                fprintf(stderr, "ERR [%d]: Could not connect to pressure sensors\n", last_err);
                return false;
            }
            m_compressor_p_ma = (float)m_compressor_p_ma_dto / 1000000.0;
            if ((last_err = industrial_dual_0_20ma_v2_get_current(&m_pressure_sensors, 0, &m_piezo_p_ma_dto)) < 0)
            {
                fprintf(stderr, "ERR [%d]: Could not connect to pressure sensors\n", last_err);
                return false;
            }
            m_piezo_p_ma = (float)m_piezo_p_ma_dto / 1000000.0;
            if ((last_err = industrial_dual_0_20ma_v2_get_current(&m_position_sensor, 1, &m_vertical_distance_ma_dto)) < 0)
            {
                fprintf(stderr, "ERR [%d]: Could not connect to LVDI position sensor\n", last_err);
                return false;
            }
            m_vertical_distance_ma = (float)m_vertical_distance_ma_dto / 1000000.0;

            industrial_analog_out_v2_create(&m_piezo_out, m_uid_p_out.c_str(), &m_ipcon);
            if ((last_err = industrial_analog_out_v2_set_enabled(&m_piezo_out, true)) < 0)
            {
                fprintf(stderr, "ERR [%d]: Could not connect to piezo regulator\n", last_err);
                return false;
            }
            m_piezo_p_ma = (float)m_piezo_p_ma_dto;

            load_cell_v2_create(&m_load_cell, m_uid_load_cell.c_str(), &m_ipcon);
            // sets the configuration to sample at 80 Hz
            // load_cell_v2_set_configuration(&m_load_cell, 1, 0);
            load_cell_v2_set_configuration(&m_load_cell, 0, 0);
            if ((last_err = load_cell_v2_get_weight(&m_load_cell, &m_weight_grams_dto)) < 0)
            {
                fprintf(stderr, "ERR [%d]: Could not connect to load cell\n", last_err);
                return false;
            }
            m_weight_grams = (float)m_weight_grams_dto;
        }
        return true;
    }

    void m_reset_time()
    {
        /*
            Resets the timer from when the object was instantiated so data can be written starting from 0 milliseconds
        */
        m_timestart = std::chrono::high_resolution_clock::now();
        return;
    }

    ~McKibbenCommander()
    {
        /*
            Safely closes all connections and disables system output
        */

        // Pressure output
        // set piezo value to zero, distable output, then destroy the obj
        industrial_analog_out_v2_set_voltage(&m_piezo_out, 0);
        industrial_analog_out_v2_set_enabled(&m_piezo_out, false);
        industrial_analog_out_v2_destroy(&m_piezo_out);

        // Brakes
        // let the brakes fall
        industrial_dual_relay_set_value(&m_brake_relays, true, false);
        this_thread::sleep_for(chrono::milliseconds(1000));
        // enable brakes then close them down
        industrial_dual_relay_set_value(&m_brake_relays, false, true);
        industrial_dual_relay_destroy(&m_brake_relays);

        // Presure sensors
        // disable sensors
        industrial_dual_0_20ma_v2_destroy(&m_pressure_sensors);
        industrial_dual_0_20ma_v2_destroy(&m_position_sensor);

        // Load cell
        load_cell_v2_destroy(&m_load_cell);

        // Shut down masters
        master_destroy(&m_master1);
        master_destroy(&m_master2);

        // Disconnect from brickd
        ipcon_destroy(&m_ipcon);
        printf("Notice: ~McKibbenCommander() - System Shutdown Successful\n");
    };

    /**
     * @brief Get the stack voltage read from master 1
     *
     * @return int - millivolts
     */
    float get_stack_voltage_1()
    {
        /*
            Stack voltage from master brick 1 - should not be used independently - later there is a function
            which returns the average from the two bricks since the data can vary slightly
        */
        if ((last_err = master_get_stack_voltage(&m_master1, &m_m1_volts_dto)) < 0)
        {
            fprintf(stderr, "ERR [%d]: Could not connect to master brick 1\n", last_err);
            return -1;
        }
        m_m1_volts = (float)m_m1_volts_dto / 1000.0;
        return m_m1_volts;
    }
    /**
     * @brief Get the stack voltage read from master 2
     *
     * @return int - millivolts
     */
    float get_stack_voltage_2()
    {
        /*
            Stack voltage from master brick 2 - should not be used independently - later there is a function
            which returns the average from the two bricks since the data can vary slightly
        */
        if ((last_err = master_get_stack_voltage(&m_master2, &m_m2_volts_dto)) < 0)
        {
            fprintf(stderr, "ERR [%d]: Could not connect to master brick 2\n", last_err);
            return -1;
        }
        m_m2_volts = (float)m_m2_volts_dto / 1000.0;
        return m_m2_volts;
    }

    /**
     * @brief Returns the average voltage between the two master bricks
     *
     * @return int - millivolts
     */
    float get_stack_voltage()
    {
        float v1 = get_stack_voltage_1();
        float v2 = get_stack_voltage_2();
        float mean = (v1 + v2) / 2;
        return mean;
    }
    /**
     * @brief Reads compressor pressure
     *
     * @return int - millivolts
     */
    float read_compressor_pressure()
    {
        if ((last_err = industrial_dual_0_20ma_v2_get_current(&m_pressure_sensors, 1, &m_compressor_p_ma_dto)) < 0)
        {
            fprintf(stderr, "ERR [%d]: Could not connect to pressure sensors\n", last_err);
            return -1;
        }
        m_compressor_p_ma = (float)m_compressor_p_ma_dto / 1000000.0;
        return m_compressor_p_ma;
    }
    /**
     * @brief Reads the pressure after the piezo regulator
     *
     * @return float - milliamps
     */
    float read_piezo_pressure()
    {
        if ((last_err = industrial_dual_0_20ma_v2_get_current(&m_pressure_sensors, 0, &m_piezo_p_ma_dto)) < 0)
        {
            fprintf(stderr, "ERR [%d]: Could not connect to pressure sensors\n", last_err);
            return -1;
        }
        m_piezo_p_ma = (float)m_piezo_p_ma_dto / 1000000.0;
        return m_piezo_p_ma;
    }
    /**
     * @brief Reads the position sensor
     *
     * @return float - milliamps
     */
    float read_position_sensor()
    {
        if ((last_err = industrial_dual_0_20ma_v2_get_current(&m_position_sensor, 1, &m_vertical_distance_ma_dto)) < 0)
        {
            fprintf(stderr, "ERR [%d]: Could not connect to LVDI position sensor\n", last_err);
            return -1;
        }
        m_vertical_distance_ma = (float)m_vertical_distance_ma_dto / 1000000.0;
        return m_vertical_distance_ma;
    }

    /**
     * @brief Reads the load cell
     *
     * @return float - grams
     */
    float read_load_cell()
    {
        if ((last_err = industrial_dual_0_20ma_v2_get_current(&m_load_cell, 0, &m_weight_grams_dto)) < 0)
        {
            fprintf(stderr, "ERR [%d]: Could not connect to LVDI position sensor\n", last_err);
            return -1;
        }
        m_weight_grams = (float)m_weight_grams_dto;
        return m_weight_grams;
    }

    /**
     * @brief Enable the brakes, returning true if successful while updating the public
     * brake state
     *
     * @return true
     * @return false
     */
    bool enable_brakes()
    {
        if ((last_err = industrial_dual_relay_set_value(&m_brake_relays, false, true)) < 0)
        {
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
    bool disable_brakes()
    {
        if ((last_err = industrial_dual_relay_set_value(&m_brake_relays, true, false)) < 0)
        {
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
    bool set_pressure_output(int millivolt_out)
    {
        if ((last_err = industrial_analog_out_v2_set_voltage(&m_piezo_out, millivolt_out)) < 0)
        {
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
    void read_all()
    {
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
    string header(string delimeter)
    {
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
    string repr(string delimiter)
    {
        char buffer[128];
        m_timenow = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed_milliseconds = m_timenow - m_timestart;
        memset(buffer, '\0', 128);
        snprintf(buffer, 128, "%0.3f%s%0.3f%s%0.3f%s%0.3f%s%0.0f%s%0.3f%s%0.0f", (float)elapsed_milliseconds.count(), delimiter.c_str(),
                 (m_m1_volts + m_m2_volts) / 2.0, delimiter.c_str(),
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
    string repr()
    {
        return repr(",");
    }

    void raise_brakes()
    {
        this->disable_brakes();
        m_timenow = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapsed_milliseconds = m_timenow - m_timestart;
        float lb = 4000.0, ub = 20000.0;
        int steps = 40;
        int duration = 3000; // ms
        int time_interval = duration / steps;
        float increments = (ub - lb) / steps;
        int wait_time = 1000;
        this_thread::sleep_for(chrono::milliseconds(wait_time));
        for (int i = lb; i < ub; i += increments)
        {
            this->set_pressure_output(i);
            this_thread::sleep_for(chrono::milliseconds(time_interval));
        }
        this_thread::sleep_for(chrono::milliseconds(wait_time));
        this->enable_brakes();
        this_thread::sleep_for(chrono::milliseconds(100));
        this->set_pressure_output(0);
    }
};

// Data transfer object

class Sequence
{
public:
    vector<float> m_times;     // times for an event to occur
    vector<float> m_magnitude; // pressures in millivolts (0 to 10000)
    uint32_t size;
    Sequence()
    {
        size = 0;
    }
    void push_back(float second, float magnitude_mv)
    {
        if (size == 0)
        {
            if (abs(second) >= 0.02)
            {
                fprintf(stderr, "Error: Sequence <push_back>: Must specify an initial condition at time = 0 seconds. Got [%f, %f]\n", second, magnitude_mv);
                exit(1);
            }
        }

        if (second > 100)
        {
            fprintf(stdout, "Warning: Test is exceeding over 100 seconds\n");
        }
        if (magnitude_mv < 0 || magnitude_mv > 20000)
        {
            fprintf(stderr, "Error: Sequence <push_back>: Magnitude in millivolts is out of bounds. Received (%f), bounds [0, 20000]\n", magnitude_mv);
            exit(1);
        }
        m_times.push_back(second);
        m_magnitude.push_back(magnitude_mv);
        size++;
    }

    void operator=(const Sequence &seq_to_copy)
    {
        m_times = seq_to_copy.m_times;
        m_magnitude = seq_to_copy.m_magnitude;
        size = seq_to_copy.size;
    }

    string repr(string delimeter)
    {
        char format_buff[64];
        string repr_result;
        for (int i = 0; i < size; i++)
        {
            memset(format_buff, '\0', 64);
            snprintf(format_buff, 64, "[%0.2f, %0.0f]", m_times[i], m_magnitude[i]);
            repr_result += string(format_buff);
            if (i != size - 1)
            {
                repr_result += delimeter;
            }
        }
        return repr_result;
    }

    string repr()
    {
        return repr(",");
    }
};

class DynamicTest
{
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
    DynamicTest(McKibbenCommander *commander)
    {
        m_commander = commander;
        m_explicit_sample_intervals = false;
        m_dt = -1;
        m_latest_nickname = "";
    };
    ~DynamicTest()
    {
        m_commander->~McKibbenCommander();
    }

    /**
     * @brief Pass two strings by reference which will be the corresponding data filename and the notes filename respectively
     *  This function will open a folder by DAY for all the experiments which were conducted if it is not open already, and then
     *  prepends this to the corresponding data directory.
     * @param data_fname
     * @param notes_fname
     */
    void get_fnames(string &data_fname, string &notes_fname, string nickname)
    {
        time_t rawtime;
        struct tm *timeinfo;
        int buff_size = 256;
        char dirname_buf[buff_size];
        char data_fname_buf[buff_size];
        char notes_fname_buf[buff_size];
        char data_fname_full_buf[buff_size];
        char notes_fname_full_buf[buff_size];
        memset(data_fname_buf, '\0', buff_size);
        memset(notes_fname_buf, '\0', buff_size);
        memset(dirname_buf, '\0', buff_size);
        memset(data_fname_full_buf, '\0', buff_size);
        memset(notes_fname_full_buf, '\0', buff_size);
        time(&rawtime);
        timeinfo = localtime(&rawtime);

        // Make a data directory is one does not exist
        if (!dirExists("data"))
        {
            if (mkdir("data", 0777) == -1)
                fprintf(stderr, "Error: DynamicTest - Cannot make directory data/- %s\n", strerror(errno));
        }

        // Make a day directory if one does not exist (all the experiments conducted in a given day)
        strftime(dirname_buf, buff_size, "data/%F", timeinfo);
        if (!dirExists(dirname_buf))
        {
            if (mkdir(dirname_buf, 0777) == -1)
                fprintf(stderr, "Error: DynamicTest - Cannot make directory data/{day}/- %s\n", strerror(errno));
        }

        // Make the exact experiment directory
        strftime(dirname_buf, buff_size, "data/%F/%F_%H%M%S", timeinfo);
        if (nickname == "")
        {
            snprintf(dirname_buf, buff_size, "%s_exp", dirname_buf);
        }
        else
        {
            snprintf(dirname_buf, buff_size, "%s_%s_exp", dirname_buf, nickname.c_str());
        }

        if (!dirExists(dirname_buf))
        {
            if (mkdir(dirname_buf, 0777) == -1)
                fprintf(stderr, "Error: DynamicTest - Cannot make directory data/{day}/*exp/- %s\n", strerror(errno));
        }
        if (nickname == "")
        {
            snprintf(data_fname_full_buf, buff_size, "%s/data.dat", dirname_buf);
            snprintf(notes_fname_full_buf, buff_size, "%s/notes.txt", dirname_buf);
        }
        else
        {
            snprintf(data_fname_full_buf, buff_size, "%s/%s_data.dat", dirname_buf, nickname.c_str());
            snprintf(notes_fname_full_buf, buff_size, "%s/%s_notes.txt", dirname_buf, nickname.c_str());
        }
        data_fname = string(data_fname_full_buf);
        notes_fname = string(notes_fname_full_buf);
    }

    void m_config_system()
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> time_zero = std::chrono::high_resolution_clock::now();
        std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = now - time_zero;
        float lag_time = 3; // magic number - 3 seconds chosen arbitrarily - can be increased if deemed safer
        printf("- - RAISING BRAKES: %0.2f seconds to cancel - abort with Ctrl-C - - -\n", lag_time);
        while ((time_span.count()) < lag_time)
        {
            printf("%0.2f s / %0.2f s\r", time_span.count(), lag_time);
            now = std::chrono::high_resolution_clock::now();
            time_span = now - time_zero;
            fflush(stdout);
        }
        printf("\n");
        fflush(stdout);
        m_commander->raise_brakes();
    }

    void m_go_to_ics(const Sequence &sequence)
    {
        std::chrono::time_point<std::chrono::high_resolution_clock> time_zero = std::chrono::high_resolution_clock::now();
        std::chrono::time_point<std::chrono::high_resolution_clock> now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = now - time_zero;
        float transition_time = 5;                // 5 seconds to reach equilibrium
        int lb = 0, ub = sequence.m_magnitude[0]; // go from resting, to the initial condition
        int steps = 40;                           // smooth the transition to initial conditions - break into 40 steps
        int action_span = transition_time * 1000; // ms
        int time_interval = action_span / steps;  // time between inputs - this is for a smooth transition rather than rapid inflation
        float increments = (ub - lb) / steps;     // the increments in which you will go from 0 to the initial condition
        int wait_time = 1000;                     // time to wait after you arrive at the destination

        // Go to initial conditions
        printf("- - Going to Initial Conditions - -\n");
        for (int i = lb; i < ub; i += (int)increments)
        {
            now = std::chrono::high_resolution_clock::now();
            time_span = now - time_zero;
            m_commander->set_pressure_output(i);
            this_thread::sleep_for(chrono::milliseconds(time_interval));
            printf("%0.2f s / %0.2f s - -\r", time_span.count(), transition_time);
            fflush(stdout);
        }
        printf("\n");
        fflush(stdout);
        // idle for some period after arrival
        this_thread::sleep_for(chrono::milliseconds(wait_time));
    }

    void init_notes(string notes_fname)
    {
        FILE *notes_ptr = fopen(notes_fname.c_str(), "a");
        time_t rawtime;
        struct tm *timeinfo;
        int buff_size = 128;
        char date_buff[buff_size];
        memset(date_buff, '\0', buff_size);
        time(&rawtime);
        timeinfo = localtime(&rawtime);
        strftime(date_buff, buff_size, "%c", timeinfo);
        for (int i = 0; i < 70; i++)
        {
            fprintf(notes_ptr, "*");
        }
        fprintf(notes_ptr, "|\n");

        fprintf(notes_ptr, "NOTES FILE FOR TEST\n\
        \rTest Conducted by: <NAME>\
        \rNickname: %s\
        \rDate: %s\
        \rUser Notes:\
        \r\n\
        \r\n\
        \r\n",
                m_latest_nickname.c_str(),
                date_buff);
        for (int i = 0; i < 70; i++)
        {
            fprintf(notes_ptr, "-");
        }
        fprintf(notes_ptr, "|\nSYSTEM OUTPUT\n");
        fclose(notes_ptr);
    }

    void term_notes(string notes_fname)
    {
        FILE *notes_ptr = fopen(notes_fname.c_str(), "a");
        fprintf(notes_ptr, "\n\n");
        for (int i = 0; i < 70; i++)
        {
            fprintf(notes_ptr, "*");
        }
        fprintf(notes_ptr, "|\n");
    }

    /**
     * @brief Provide a step input to the system
     *
     * @param step_sequence Sequence of events times and desired magnitudes
     * @param dt A specified sample interval - use -1 for fastest, while anything >0 for a specific interval - may result in unwanted consequences
     * @param nickname
     * @param verbose
     * @param update_frq
     */
    void run_step(Sequence step_sequence, float dt, string nickname, bool verbose, float update_frq)
    {
        m_latest_nickname = nickname;
        m_latest_sequence = step_sequence;
        // Check inputs
        if (step_sequence.size < 2)
        {
            fprintf(stderr, "Error: DynamicTest <run_step> - Sequence must contain at least two members\n");
        }
        if (dt < 0)
        {
            m_explicit_sample_intervals = false;
        }
        else
        {
            if (dt < 0.004 && dt >= 0)
            {
                fprintf(stdout, "Warning: DynamicTest <run_step> - Sample time is less than fastest observed sample time of 0.004 seconds (4 ms/250 Hz). Received %f\n", dt);
            }
            m_explicit_sample_intervals = true;
            m_dt = dt * 1000; // seconds to milliseconds
        }
        printf("Notice: Dynamic Test <run_step> Selected\n");
        FILE *data_fptr;
        FILE *notes_fptr;
        string data_fname, notes_fname;
        get_fnames(data_fname, notes_fname, nickname);

        // Raise brakes and give the user time to respond
        m_config_system();

        // Go to initial value and hold for some number of seconds to reach an equilibrium
        m_go_to_ics(step_sequence);
        std::chrono::time_point<std::chrono::high_resolution_clock> time_zero, now, next_sample;
        time_zero = std::chrono::high_resolution_clock::now();
        now = time_zero;
        next_sample = time_zero;
        std::chrono::duration<double> time_span = now - time_zero; // general case

        double start_time = time_span.count();

        printf("- - Test In Progress - -\n");
        m_commander->m_reset_time();
        data_fptr = fopen(data_fname.c_str(), "a");

        fprintf(data_fptr, "%s\n", m_commander->header(",").c_str());
        init_notes(notes_fname);
        notes_fptr = fopen(notes_fname.c_str(), "a");
        fprintf(notes_fptr, "Sequence: \n\t-%s", step_sequence.repr("\n\t-").c_str());
        fclose(notes_fptr);
        /*
            Here, we expect that a sequence is provided by event driven time steps. I.e. if a sequence time/mag pair is [10, 10000],
            at 10 seconds we will provide a 10000 mA input to the system

            We iterate over each of these sequences and hold at the position capturing/reading the data until the next event
        */

        float hardware_timer_err = 0, accumulated_err = 0;
        ;
        float pre_time = 0;
        float post_time = 0;
        float next_update = 0;
        uint64_t samples = 1;
        // iterate over each pair in the sequence, and hold a magnitude until the next event is reached
        for (int i = 0; i <= step_sequence.size; i++)
        {
            float t = 0;
            if (i != step_sequence.size - 1)
            {
                t = step_sequence.m_times[i + 1];
            }
            int mag = (int)step_sequence.m_magnitude[i];
            bool sig_out = false;
            // hold until the next event is reached
            // less safe to keep the file opened throughout the entire event, but will result in a faster sampling rate - where if the test is disrupted the data is likely bad anyways for the scope of these tests
            // to comprimise, we open and close it between these blocks
            data_fptr = fopen(data_fname.c_str(), "a");
            while ((time_span.count()) < start_time + t)
            {
                m_commander->read_all();
                pre_time = time_span.count();
                fprintf(data_fptr, "%s\n", m_commander->repr().c_str());
                now = std::chrono::high_resolution_clock::now();
                time_span = now - time_zero;

                // flip once - cleaner/saves comms bandwidth
                if (!sig_out)
                {
                    m_commander->set_pressure_output(mag);
                    sig_out = true;
                }
                if (m_explicit_sample_intervals)
                {
                    // we time the difference in time between read/writes - and if this is slower than the desired wait time consistently
                    //  we reduce how long we wait by this error
                    hardware_timer_err = (accumulated_err / samples);
                    this_thread::sleep_for(chrono::milliseconds(m_dt + (int)hardware_timer_err));
                    now = std::chrono::high_resolution_clock::now();
                    time_span = now - time_zero;
                    post_time = time_span.count();
                    accumulated_err += (m_dt - (post_time - pre_time) * 1000);
                }
                samples++;
                if (verbose && time_span.count() > next_update)
                {
                    printf("%0.2f s / %0.2f s\r", time_span.count(), step_sequence.m_times.back());
                    fflush(stdout);
                    next_update = time_span.count() + update_frq;
                }
            }
            fclose(data_fptr);
            m_commander->set_pressure_output(0);
        }
        printf("\n");
        term_notes(notes_fname);
        printf("- - TEST DONE - -\n");
    }

    /**
     * @brief Provide a ramp input to the system
     *  Warning - to maintain maximum sampling frequency, we have to make some approximations
     * We break the input into 100 steps, and also the estimated time between steps. We hold at a signal step until the signal step time has reached the
     * fixed estimated time between signals. We correct this by looking to see the deviation between the desired slope and effective slope. With this, you will notice
     * that your actual time may slightly exceed your desired time. Currently a low priority fix.
     *
     * @param ramp_sequence Sequence of times and magnitudes - steps are broken into 100 steps
     * @param dt -1 if you want the fastest sample rate, > 0 if you want to impose a specific sample rate
     * @param nickname A nickname to appear in the files
     * @param verbose Print realtime progress output
     * @param update_frq Frequency to receive updates (if you are in verbose mode)
     */
    void run_ramp(Sequence ramp_sequence, float dt, string nickname, bool verbose, float update_frq)
    {
        m_latest_nickname = nickname;
        m_latest_sequence = ramp_sequence;
        // Check inputs
        if (ramp_sequence.size < 2)
        {
            fprintf(stderr, "Error: DynamicTest <run_ramp> - Sequence must contain at least two members\n");
        }
        if (dt < 0)
        {
            m_explicit_sample_intervals = false;
        }
        else
        {
            if (dt < 0.004 && dt >= 0)
            {
                fprintf(stdout, "Warning: DynamicTest <run_ramp> - Sample time is less than fastest observed sample time of 0.004 seconds (4 ms/250 Hz). Received %f\n", dt);
            }
            m_explicit_sample_intervals = true;
            m_dt = dt * 1000; // seconds to milliseconds
        }
        printf("Notice: Dynamic Test <run_ramp> Selected\n");
        FILE *data_fptr;
        FILE *notes_fptr;
        string data_fname, notes_fname;
        get_fnames(data_fname, notes_fname, nickname);

        // Raise brakes and give the user time to respond
        m_config_system();

        // Go to initial value and hold for some number of seconds to reach an equilibrium
        m_go_to_ics(ramp_sequence);
        std::chrono::time_point<std::chrono::high_resolution_clock> time_zero, now, block_start, block_time, next_sample;
        time_zero = std::chrono::high_resolution_clock::now();
        block_start = time_zero;
        block_time = time_zero;
        now = time_zero;
        next_sample = time_zero;
        std::chrono::duration<double> time_span = now - time_zero, block_span = now - time_zero;

        double start_time = time_span.count();

        printf("- - Test In Progress - -\n");
        m_commander->m_reset_time();
        data_fptr = fopen(data_fname.c_str(), "a");

        fprintf(data_fptr, "%s\n", m_commander->header(",").c_str());
        fclose(data_fptr);
        init_notes(notes_fname);
        notes_fptr = fopen(notes_fname.c_str(), "a");
        fprintf(notes_fptr, "Sequence: \n\t-%s", ramp_sequence.repr("\n\t-").c_str());
        fclose(notes_fptr);
        /*
            Here, we expect that a sequence is provided by event driven time steps. I.e. if a sequence time/mag pair is [10, 10000],
            at 10 seconds we will provide a 10000 mA input to the system

            We iterate over each of these sequences and hold at the position capturing/reading the data until the next event
        */

        float hardware_timer_err = 0, accumulated_err = 0;
        float pre_time = 0;
        float post_time = 0;
        float next_update = 0;
        uint64_t samples = 1;
        // iterate over each pair in the sequence, and hold a magnitude until the next event is reached
        for (int i = 0; i < ramp_sequence.size; i++)
        {
            float t = 0;
            if (i != ramp_sequence.size - 1)
            {
                t = ramp_sequence.m_times[i + 1];
            }
            int mag = (int)ramp_sequence.m_magnitude[i];
            bool sig_out = false;
            // hold until the next event is reached
            // less safe to keep the file opened throughout the entire event, but will result in a faster sampling rate - where if the test is disrupted the data is likely bad anyways for the scope of these tests
            // to comprimise, we open and close it between these blocks
            data_fptr = fopen(data_fname.c_str(), "a");

            /*
                FORMULATION OF RAMPS
                Go from the initial magnitude to the next magnitude if one exists
                This will be broken into some number of increments to get to that point
            */
            float init = ramp_sequence.m_magnitude[i], fv;
            float init_time = ramp_sequence.m_times[i], fv_time;
            if (i != ramp_sequence.size - 1)
            {
                fv = ramp_sequence.m_magnitude[i + 1];
                fv_time = ramp_sequence.m_times[i + 1];
            }
            else
            {
                fv = ramp_sequence.m_magnitude[i];
                fv = ramp_sequence.m_times[i];
            }
            int n_steps = 100; // magic number, chosen arbitrarily - something that should be chosen explicitly however so testing is consistent due to hardware limitations

            float increment = ceil((float)(fv - init) / (float)n_steps);
            float time_step = (fv_time - init_time) / (float)n_steps;
            float slope_wrt_time = (fv - init) / (fv_time - init_time);
            int sign_flip = (fv - init) / abs(fv - init); // returns +- 1 for if the slope is positive or negative
            float eps = 0.001;
            // run until we are diverging from the goal
            float correction_factor = 1;
            for (float sig = init; sign_flip * (fv - sig) > sign_flip * eps; sig += increment)
            {
                // this block should take exactly 1 time step to complete, if it does not, then we hold and continue reading
                // TODO: Include timer for this problem - currently we established how to increment signals, but not how to manage time
                m_commander->set_pressure_output((int)sig);
                m_commander->read_all();
                pre_time = time_span.count();
                fprintf(data_fptr, "%s\n", m_commander->repr().c_str());
                now = std::chrono::high_resolution_clock::now();
                block_start = std::chrono::high_resolution_clock::now();
                block_time = std::chrono::high_resolution_clock::now();
                time_span = now - time_zero;
                block_span = block_time - now;
                if (m_explicit_sample_intervals)
                {
                    // we time the difference in time between read/writes - and if this is slower than the desired wait time consistently
                    //  we reduce how long we wait by this error
                    hardware_timer_err = (accumulated_err / samples);
                    this_thread::sleep_for(chrono::milliseconds(m_dt + (int)hardware_timer_err));
                    now = std::chrono::high_resolution_clock::now();
                    time_span = now - time_zero;
                    post_time = time_span.count();
                    accumulated_err += (m_dt - (post_time - pre_time) * 1000);
                }
                samples++;
                if (verbose && time_span.count() > next_update)
                {
                    printf("%0.2f s / %0.2f s - Signal: %f\r", time_span.count(), ramp_sequence.m_times.back(), sig);
                    fflush(stdout);
                    next_update = time_span.count() + update_frq;
                }
                while (block_span.count() < time_step * correction_factor * correction_factor)
                {
                    block_time = std::chrono::high_resolution_clock::now();
                    now = std::chrono::high_resolution_clock::now();
                    block_span = block_time - block_start;
                    time_span = now - time_zero;
                    /*
                        This read_all function has been observed to take ~5-10 ms - when it was left out this block could write the data
                        (stale data) at extremely high frequencies
                    */
                    m_commander->read_all();
                    fprintf(data_fptr, "%s\n", m_commander->repr().c_str());
                }
                /*
                    We have a correction factor to account for the difference between our actual slope and effectively slope -
                    it was observed that we would not complete the sequence in time, therefore if we are undershooting our target, we need to
                    spend less time in the block above, and reduce the timestep.
                    We take the percent error between the desired slope and the effective slope, where if the effective slope is less than desired slope,
                    we end up with a smaller number
                */
                if (abs((fv - sig) / (fv - init)) < 0.95)
                {
                    // we find the value 5 seconds into the test (15-10)*slope

                    float effective_slope = (sig - init) / (time_span.count() - ramp_sequence.m_times[i]); // effective slope from initial conditions
                    // printf("s %f es %f tdiff %f\n", slope_wrt_time, effective_slope, (time_span.count()-ramp_sequence.m_times[i]));
                    correction_factor = 1 - (slope_wrt_time - effective_slope) / slope_wrt_time; // this reduces the timestep to catch up to the desired slope
                    // printf("cf: %f, %f\n", correction_factor, abs((fv - sig)/(fv - init)));
                }
            }
            fclose(data_fptr);
            m_commander->set_pressure_output(0);
        }
        printf("\n");
        term_notes(notes_fname);
        printf("- - TEST DONE - -\n");
    }

    void run_frq_response(float center_pressure, float amplitude, float frq_rads, float duration, string nickname, bool verbose, float update_frq)
    {
        /*
            FREQUENCY RESPONSE TEST
            Here we are to observe histeresis in the actuator, as well as the thermo dynamic response when subject to fast oscillations for an extended period of time
            The expectation for what this function will do, is you provide a single frequency, a centered pressure, an amplitude, and a duration, st.
            It provides these oscillations for some period of time
        */
        m_latest_nickname = nickname;
        printf("Notice: Dynamic Test <run_frq_response> Selected\n");
        FILE *data_fptr;
        FILE *notes_fptr;
        string data_fname, notes_fname;
        get_fnames(data_fname, notes_fname, nickname);

        // Raise brakes and give the user time to respond
        m_config_system();

        // Go to initial value and hold for some number of seconds to reach an equilibrium
        Sequence pseu_sequence;
        pseu_sequence.push_back(0, center_pressure);
        m_go_to_ics(pseu_sequence);

        std::chrono::time_point<std::chrono::high_resolution_clock> time_zero, now, next_sample;
        time_zero = std::chrono::high_resolution_clock::now();
        now = time_zero;
        next_sample = time_zero;
        std::chrono::duration<double> time_span = now - time_zero; // general case

        double start_time = time_span.count();

        printf("- - Test In Progress - -\n");
        m_commander->m_reset_time();
        data_fptr = fopen(data_fname.c_str(), "a");

        fprintf(data_fptr, "%s\n", m_commander->header(",").c_str());
        init_notes(notes_fname);
        notes_fptr = fopen(notes_fname.c_str(), "a");
        fprintf(notes_fptr, "Frequency response parameters:\
            \r\t- Frequency: %0.3f rad/s\
            \r\t- Center Pressure: %0.1f mV\
            \r\t- Amplitude: %0.1f mV\
            \r\t- Duration: %0.2f s\n", 
            frq_rads, 
            center_pressure, 
            amplitude, 
            duration);

        fclose(notes_fptr);
        /*
            Here, we expect that a sequence is provided by event driven time steps. I.e. if a sequence time/mag pair is [10, 10000],
            at 10 seconds we will provide a 10000 mA input to the system

            We iterate over each of these sequences and hold at the position capturing/reading the data until the next event
        */

        float hardware_timer_err = 0, accumulated_err = 0;
        ;
        float pre_time = 0;
        float post_time = 0;
        float next_update = 0;
        uint64_t samples = 1;
        // iterate over each pair in the sequence, and hold a magnitude until the next event is reached
        data_fptr = fopen(data_fname.c_str(), "a");
        while(time_span.count() < duration) {
            m_commander->read_all();
            fprintf(data_fptr, "%s\n", m_commander->repr().c_str());
            now = std::chrono::high_resolution_clock::now();
            time_span = now - time_zero;
            float mag_out = center_pressure+amplitude*sin(frq_rads*time_span.count());
            m_commander->set_pressure_output((int)mag_out);
            if (verbose && time_span.count() > next_update)
            {
                printf("%0.2f s / %0.2f s\r", time_span.count(), duration);
                fflush(stdout);
                next_update = time_span.count() + update_frq;
            }
        }
        fclose(data_fptr);
        m_commander->set_pressure_output(0);
        printf("\n");
        term_notes(notes_fname);
        printf("- - TEST DONE - -\n");
    }
};