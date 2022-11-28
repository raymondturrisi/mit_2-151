"""
   Data Acquisition for Interchangeable Hardware
   Electrolytic Cell
   Author:  PGM
   Revision:  2022-06-02
   
   Modified
   21 Jun 2022 - added conductivity probes
   22 Jun 2022 - added polarization measurement to end of exp series
   27 Jun 2022 - added anode pump stepper motor control  
   25 Jul 2022 - added userProceed input, swapped anode/cathode conductivity UID's
   10 Aug 2022 - upgrades: 8x valves, Pt RTD's, Load cells
"""

# import required packages
import numpy as np
import os
from linResample import *
from sampleStats import *
from writeCSV import *
from readCSV import *
from logLine import *
from digilent_Aout import *
from readExperimentConfig import *

# GET TINKERFORGE COMPONENTS
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_silent_stepper import BrickSilentStepper
from tinkerforge.bricklet_voltage_current_v2 import BrickletVoltageCurrentV2
from tinkerforge.bricklet_thermocouple_v2 import BrickletThermocoupleV2
from tinkerforge.bricklet_analog_in_v3 import BrickletAnalogInV3
from tinkerforge.bricklet_industrial_ptc import BrickletIndustrialPTC
from tinkerforge.bricklet_industrial_quad_relay_v2 import BrickletIndustrialQuadRelayV2
from tinkerforge.bricklet_load_cell_v2 import BrickletLoadCellV2

# DEFINE TINKERFORGE HARDWARE ADDR
HOST = "localhost"
PORT = 4223
# outputs
UID_ssc = "6EGxZK" # cathode pump
UID_ssa ="6EGdvP" # anode pump
UID_iqr14 = "J42" # quad relay outputs for V1-V4
UID_iqr58 = "21yD" # qaud relay outputs for V5-V8
# inputs
UID_vc = "PE6"      # cell voltage and current
#UID_anode = "Ms4" # anode temp
#UID_cathode = "Msk" # cathode temp
UID_master1 = "6a1ZuC" # master brick
UID_kc = "KnC" # cathode conductivity
UID_ka = "KnA" # anode conductivity 
UID_rtda1 = "TM1" # anode temperature
UID_rtdc1 = "TNh" # cathode temperature
UID_lca1 = "Zjp" # milk load cell
UID_lca2 = "VHs" # DIW load cell
UID_lca3 = "Zjr" # spare load cell

imodTime = 0 # modulus variable for real time display of data
logLines = [] # list for logging and printing to console

# BEGIN USER CONFIGURATION 
# file handling, root directory plus file
directory = os.path.abspath(os.path.join(os.path.curdir)) # working directory root
filepathP = directory+'/customWaveforms/polarization.csv' 
filepathR = directory+'/customWaveforms/recovery.csv' 
filepathE1 = directory+'/customWaveforms/recovery.csv' 
filepathN = directory+'/customWaveforms/recovery.csv'

# read experiment configuration data
filename = directory+'/experimentConfig/test_Short.csv' # root directory plus file
[expList, hzFreq, cSamples, amp, offset, repeatCount, waitTime, modeAConfig, modeCConfig, \
    pumpA1, pumpC1, userProceed] = readExperimentConfig(filename)

# tinkerforge configuration
dt= 0.2 # change this to desired intersample interval e.g., 1.0 s
printTime = 10 # time in seconds between print statements

# valve alignment modes
modeA0 = [0,0,0,0,0,0,0,0] # all closed
modeA1 = [1,0,0,0,0,1,0,0] # milk: V1, V6 open
modeA2 = [0,0,1,1,0,0,0,0] # DIW: V3, V4 open
modeA3 = [1,0,0,0,1,0,0,0] # air to milk forward: V5, V1 open
modeA4 = [0,1,0,0,0,1,0,0] # air to milk reverse: V2, V6 open
modeA5 = [0,0,1,0,1,0,0,0] # air to DIW forward: V5, V3 open
modeA6 = [0,1,0,1,0,0,0,0] # air to DIW reverse: V2, V4 open
modeC0 = [0,0,0,0,0,0,0,0] # V7, V8 closed
modeC1 = [0,0,0,0,0,0,1,1] # cathode rinse: V7, V8 open

# END USER CONFIGURATION

# CALLBACK FUNCTIONS
# define callback function for voltage/current 2.0
def cb_current(current):
    global iVal
    global iTime
    global iIndex
    global n
    global dt

    global imodTime
    global printTime
    
    if iIndex < n:
        iTime[iIndex] = time.perf_counter_ns()
        iVal[iIndex] = current/1000.0 # Amps

        # increment the index to keep track of number of samples collected for post processing
        iIndex = iIndex + 1  
        # statement to print values every printTime seconds
        if iIndex % (1/dt) == 0: 
            imodTime = imodTime+1
            if imodTime == printTime:
                imodTime = 0
                realtimeVal()

        # INSERT HERE - perform temperature control update

# define callback function for voltage/current 2.0
def cb_voltage(voltage):
    global eVal
    global eTime
    global eIndex
    global n
    
    if eIndex < n:
        eTime[eIndex] = time.perf_counter_ns()
        eVal[eIndex] = voltage/1000.0 # Volts

        # increment the index to keep track of number of samples collected for post processing
        eIndex = eIndex + 1 
           
# define callback function for thermocouple
def cb_tca(temp):
    global tcaVal
    global tcaTime
    global tcaIndex
    global n

    if tcaIndex < n:
        tcaTime[tcaIndex] = time.perf_counter_ns()
        tcaVal[tcaIndex] = temp/100.0 # C

        # increment the index to keep track of number of samples collected for post processing
        tcaIndex = tcaIndex + 1
        
# define callback function for thermocouple
def cb_tcc(temp):
    global tccVal
    global tccTime
    global tccIndex
    global n
    
    if tccIndex < n:
        tccTime[tccIndex] = time.perf_counter_ns()
        tccVal[tccIndex] = temp/100.0 # C

        # increment the index to keep track of number of samples collected for post processing
        tccIndex = tccIndex + 1 

# define callback function for analog in - conductivity probe for Cathode 
def cb_kc(voltage):
    global kcVal
    global kcTime
    global kcIndex
    global n
    
    if kcIndex < n:
        kcTime[kcIndex] = time.perf_counter_ns()
        kcVal[kcIndex] = voltage/1000.0 # Volts

        # increment the index to keep track of number of samples collected for post processing
        kcIndex = kcIndex + 1 

# define callback function for analog in - conductivity probe for Anode 
def cb_ka(voltage):
    global kaVal
    global kaTime
    global kaIndex
    global n
    
    if kaIndex < n:
        kaTime[kaIndex] = time.perf_counter_ns()
        kaVal[kaIndex] = voltage/1000.0 # Volts

        # increment the index to keep track of number of samples collected for post processing
        kaIndex = kaIndex + 1 
        
# define callback function for PTC bricklet - cathode RTD-A1
def cb_rtda1(temperature):
    global rtda1Val
    global rtda1Time
    global rtda1Index
    global n
    
    if rtda1Index < n:
        rtda1Time[rtda1Index] = time.perf_counter_ns()
        rtda1Val[rtda1Index] = temperature/100.0 # deg C

        # increment the index to keep track of number of samples collected for post processing
        rtda1Index = rtda1Index + 1 

# define callback function for PTC bricklet - cathode RTD-C1
def cb_rtdc1(temperature):
    global rtdc1Val
    global rtdc1Time
    global rtdc1Index
    global n
    
    if rtdc1Index < n:
        rtdc1Time[rtdc1Index] = time.perf_counter_ns()
        rtdc1Val[rtdc1Index] = temperature/100.0 # deg C

        # increment the index to keep track of number of samples collected for post processing
        rtdc1Index = rtdc1Index + 1 

# define callback function for milk load cell - LC-A1
def cb_lca1(weight):
    global lca1Val
    global lca1Time
    global lca1Index
    global n
    
    if lca1Index < n:
        lca1Time[lca1Index] = time.perf_counter_ns()
        lca1Val[lca1Index] = weight # g

        # increment the index to keep track of number of samples collected for post processing
        lca1Index = lca1Index + 1 

# define callback function for DIW load cell - LC-A2
def cb_lca2(weight):
    global lca2Val
    global lca2Time
    global lca2Index
    global n
    
    if lca2Index < n:
        lca2Time[lca2Index] = time.perf_counter_ns()
        lca2Val[lca2Index] = weight # g

        # increment the index to keep track of number of samples collected for post processing
        lca2Index = lca2Index + 1 

# define callback function for spare load cell - LC-A3
def cb_lca3(weight):
    global lca3Val
    global lca3Time
    global lca3Index
    global n
    
    if lca3Index < n:
        lca3Time[lca3Index] = time.perf_counter_ns()
        lca3Val[lca3Index] = weight # g

        # increment the index to keep track of number of samples collected for post processing
        lca3Index = lca3Index + 1 

# OTHER FUNCTIONS    
# Configure tinkerforge samping rates 
def setSamplingRate():
    
    global logLines
    
    # Set sample rates
    vc.set_configuration(3,4,4) # set_configuration(averaging, voltage_conversion_time, current_conversion_time), default = AVERAGING_64 = 3, CONVERSION_TIME_1_1MS = 4, CONVERSION_TIME_1_1MS = 4
    #tca.set_configuration(16,3,1) # set_configuration(averaging, thermocouple_type, filter), default = AVERAGING_16 = 16, TYPE_K = 3, FILTER_OPTION_60HZ = 1
    #tcc.set_configuration(16,3,1) # set_configuration(averaging, thermocouple_type, filter), default = AVERAGING_16 = 16, TYPE_K = 3, FILTER_OPTION_60HZ = 1
    #master1.set_spitfp_baudrate_config(0, 2000000) # enable_dynamic_baudrate: True/False, minimum_dynamic_baudrate: range [400000 to 2000000] 
    #master1.set_spitfp_baudrate('d', 2000000) #bricklet_port = Type: chr, Range: ["a" to "d"], baudrate = Type: int, Unit: 1 Bd, Range: [400000 to 2000000], Default: 1400000
    kc.set_oversampling(32) # sample every 17.5us * oversampling, i.e. 17.5us*32=0.56ms
    ka.set_oversampling(32) # sample every 17.5us * oversampling, i.e. 17.5us*32=0.56ms
    rtda1.set_wire_mode(4) # 4 wire mode
    rtda1.set_moving_average_configuration(1, 1) # 1=averaging off, first value is for resistance, second value for temperature
    rtdc1.set_wire_mode(4) # 4 wire mode
    rtdc1.set_moving_average_configuration(1, 1) # 1=averaging off, first value is for resistance, second value for temperature
    lca1.set_moving_average(1) # 1=averaging off
    lca1.set_configuration(0, 0), # RATE_10HZ = 0, GAIN_128X = 0 (2mV/V)
    lca2.set_moving_average(1) # 1=averaging off
    lca2.set_configuration(0, 0), # RATE_10HZ = 0, GAIN_128X = 0 (2mV/V)
    #lca3.set_moving_average(1) # 1=averaging off
    #lca3.set_configuration(0, 0), # RATE_10HZ = 0, GAIN_128X = 0 (2mV/V)

    logLines = logLine(logLines, "vc config = "+str(vc.get_configuration()))
    #logLines = logLine(logLines, "tca config = "+str(tca.get_configuration()))
    #logLines = logLine(logLines, "tcc config = "+str(tcc.get_configuration()))
    logLines = logLine(logLines, "kc config = "+str(kc.get_oversampling()))
    logLines = logLine(logLines, "ka config = "+str(ka.get_oversampling()))
    #print("master1 baude rate = "+str(master1.get_spitfp_baudrate_config()))
    #print("port d baude rate = "+str(master1.get_spitfp_baudrate('d'))) 
    logLines = logLine(logLines, "rtda1 connected = "+str(rtda1.is_sensor_connected()))
    logLines = logLine(logLines, "rtda1 wire mode = "+str(rtda1.get_wire_mode()))
    logLines = logLine(logLines, "rtda1 moving average config = "+str(rtda1.get_moving_average_configuration()))
    logLines = logLine(logLines, "rtdc1 connected = "+str(rtdc1.is_sensor_connected()))
    logLines = logLine(logLines, "rtdc1 wire mode = "+str(rtdc1.get_wire_mode()))
    logLines = logLine(logLines, "rtdc1 moving average config = "+str(rtdc1.get_moving_average_configuration()))
    logLines = logLine(logLines, "lca1 moving average config = "+str(lca1.get_moving_average()))
    logLines = logLine(logLines, "lca1 rate and gain = "+str(lca1.get_configuration()))
    logLines = logLine(logLines, "lca2 moving average config = "+str(lca2.get_moving_average()))
    logLines = logLine(logLines, "lca2 rate and gain = "+str(lca2.get_configuration()))
    #logLines = logLine(logLines, "lca3 moving average config = "+str(lca3.get_moving_average()))
    #logLines = logLine(logLines, "lca3 rate and gain = "+str(lca3.get_configuration()))

def realtimeVal():
    global iVal
    global iIndex
    global eVal
    global eIndex
    #global tcaVal
    #global tcaIndex
    #global tccVal
    #global tccIndex
    global kcVal
    global kcIndex
    global kaVal
    global kaIndex
    global rtda1Val
    global rtda1Index
    global rtdc1Val
    global rtdc1Index
    global lca1Val
    global lca1Index
    global lca2Val
    global lca2Index
    #global lca3Val
    #global lca3Index
    global logLines

    logLines = logLine(logLines,"----------------")
    logLines = logLine(logLines,"current = "+str(iVal[iIndex-1]))
    logLines = logLine(logLines,"voltage = "+str(eVal[eIndex-1]))
    #logLines = logLine(logLines, "Anode Temp = "+str(tcaVal[tcaIndex-1]))    
    #logLines = logLine(logLines, "Cathode Temp = "+str(tccVal[tccIndex-1]))
    logLines = logLine(logLines, "cathode conductivity = "+str(kcVal[kcIndex-1]))
    logLines = logLine(logLines, "anode conductivity = "+str(kaVal[kaIndex-1]))
    logLines = logLine(logLines, "anode temp = "+str(rtda1Val[rtda1Index-1]))   
    logLines = logLine(logLines, "cathode temp = "+str(rtdc1Val[rtdc1Index-1]))    
    logLines = logLine(logLines, "milk mass = "+str(lca1Val[lca1Index-1])) 
    logLines = logLine(logLines, "DIW mass = "+str(lca2Val[lca2Index-1]))  
    #logLines = logLine(logLines, "spare mass = "+str(lca3Val[lca3Index-1]))   
    
def numSamples(hzFreq):
    # caclulate the number of samples based on length of digilent signal 
    T=1.0/hzFreq
    T = np.ceil(T)
    T = T.astype(int)
    n = T/dt
    n=n.astype(int) # number of samples for tinkerforge
    return n

# BEGIN MAIN LOOP
if __name__ == "__main__":
    
    i = 0
    iter = len(expList)
    loopTimestamp = np.zeros(iter) # to save start time of each loop

    while i<iter: 
        logLines = logLine(logLines,"%%%%%%%%%% start iteration "+str(i)+" %%%%%%%%%%")
        
        # program waits for user input to proceed 
        print("User proceed setting: "+str(userProceed[i]))
        if userProceed[i]:
           proceed = False
           while not proceed:
                userinput = input("press Y + Enter:")
                print("key pressed:"+ userinput)
                if userinput == "Y":
                    proceed = True
        print("proceeding with interation "+str(i))

        loopTimestamp[i]=time.perf_counter_ns()*1e-9 # in seconds
        loopSec = np.trunc(loopTimestamp[i]-loopTimestamp[0])
        logLines = logLine(logLines,"experiment elapsed time "+str(loopSec)+" sec")

        # caclulate the number of samples based on Digilent signal
        n = numSamples(hzFreq[i])
        
        # intiailze measurement arrays
        #tcaVal = np.zeros(n) # anode temperature
        #tccVal = np.zeros(n) # cathode temperature
        eVal = np.zeros(n) # voltage
        iVal = np.zeros(n) # current
        kcVal = np.zeros(n) # cathode conductivity
        kaVal = np.zeros(n) # anode conductivity
        rtda1Val = np.zeros(n) # anode temperature
        rtdc1Val = np.zeros(n) # cathode temperature
        lca1Val = np.zeros(n) # milk mass
        lca2Val = np.zeros(n) # DIW mass
        #lca3Val = np.zeros(n) # spare load cell mass
    
        # timestamps
        #tcaTime = np.zeros(n) 
        #tccTime = np.zeros(n) 
        eTime = np.zeros(n) 
        iTime = np.zeros(n)
        kcTime = np.zeros(n)
        kaTime = np.zeros(n) 
        rtda1Time = np.zeros(n) 
        rtdc1Time = np.zeros(n) 
        lca1Time = np.zeros(n) 
        lca2Time = np.zeros(n) 
        #lca3Time = np.zeros(n) 
        
        # sample indexes
        #tcaIndex = 0
        #tccIndex = 0
        eIndex = 0
        iIndex = 0
        kcIndex = 0
        kaIndex = 0
        rtda1Index = 0
        rtdc1Index = 0
        lca1Index = 0
        lca2Index = 0
        #lca3Index = 0

        logLines = logLine(logLines,"connecting to tinkerforge hardware...")    
        ipcon = IPConnection() # Create IP connection
        # outputs
        ssc = BrickSilentStepper(UID_ssc, ipcon) # Create device object
        ssa = BrickSilentStepper(UID_ssa, ipcon) # Create device object
        iqr14 = BrickletIndustrialQuadRelayV2(UID_iqr14, ipcon) # Create device object
        iqr58 = BrickletIndustrialQuadRelayV2(UID_iqr58, ipcon) # Create device object
        # inputs
        vc = BrickletVoltageCurrentV2(UID_vc, ipcon) # Create device object
        #tca = BrickletThermocoupleV2(UID_anode, ipcon) # Create device object
        #tcc = BrickletThermocoupleV2(UID_cathode, ipcon) # Create device object
        kc = BrickletAnalogInV3(UID_kc, ipcon) # Create device object
        ka = BrickletAnalogInV3(UID_ka, ipcon) # Create device object
        #master1 = BrickSilentStepper(UID_master1, ipcon) # Create device object
        rtda1 = BrickletIndustrialPTC(UID_rtda1, ipcon) # Create device object
        rtdc1 = BrickletIndustrialPTC(UID_rtdc1, ipcon) # Create device object
        lca1 = BrickletLoadCellV2(UID_lca1, ipcon) # Create device object
        lca2 = BrickletLoadCellV2(UID_lca2, ipcon) # Create device object
        #lca3 = BrickletLoadCellV2(UID_lca3, ipcon) # Create device object

        ipcon.connect(HOST, PORT) # Connect to brick
        # don't use device before ipcon is connected
        time.sleep(0.2)
        logLines = logLine(logLines,"done connecting")
        # show initial measurements
        voltage = vc.get_voltage()
        logLines = logLine(logLines,"voltage: " + str(voltage/1000.0) + " V")
        # show initial current
        current = vc.get_current()
        logLines = logLine(logLines,"current: " + str(current/1000.0) + " A")
        # show initial cathode conductivity
        kc_now = kc.get_voltage()
        logLines = logLine(logLines,"cathode conductivity(raw): " + str(kc_now/1000.0) + " V")
        # show initial anode conductivity
        ka_now = ka.get_voltage()
        logLines = logLine(logLines,"anode conductivity(raw): " + str(ka_now/1000.0) + " V")
        # show initial anode temperature from RTD-A1
        rtda1_now = rtda1.get_temperature()
        logLines = logLine(logLines,"anode temp from RTD-A1: " + str(rtda1_now/100.0) + " C")
        # show initial cathode temperature from RDT-C1
        rtdc1_now = rtdc1.get_temperature()
        logLines = logLine(logLines,"cathode temp from RTD-C1: " + str(rtdc1_now/100.0) + " C")
        # show initial milk mass from LC-A1
        lca1_now = lca1.get_weight()
        logLines = logLine(logLines,"milk mass from LC-A1: " + str(lca1_now) + " g")
        # show initial DIW mass from LC-A2
        lca2_now = lca2.get_weight()
        logLines = logLine(logLines,"DIW mass from LC-A2: " + str(lca2_now) + " g")
        # show initial mass from LC-A3
        # lca3_now = lca3.get_weight()
        # logLines = logLine(logLines,"spare mass from LC-A3: " + str(lca3_now) + " g")
        
        # update valve alignment based on operating mode
        period = 1.0/hzFreq[i]
        modeNow = np.bitwise_or(modeAConfig[i],modeCConfig[i])
        j=0
        while j<len(modeNow):
            if modeNow[j]:
                logLines = logLine(logLines,"opening valve V-"+str(j+1))
                if j<4:
                    #iqr14.set_monoflop(j,True, period) # open valve for period of time]
                    iqr14.set_selected_value(j,True) # open valve 
                else:
                    #iqr58.set_monoflop(j,True, period) # open valve for period of time]
                    iqr58.set_selected_value(j,True) # open valve 
            else:
                logLines = logLine(logLines,"closing valve V-"+str(j+1))
                if j<4:
                    #iqr14.set_monoflop(j,False, period) # close valve for period of time
                    iqr14.set_selected_value(j,False) # close valve 
                else:
                    #iqr58.set_monoflop(j,False, period) # close valve for period of time
                    iqr58.set_selected_value(j,False) # close valve 
            j+=1
        
        # pump motor logic, configure stepper motor parameters
        ssa.set_motor_current(800) # 800 mA
        ssa.set_step_configuration(ssa.STEP_RESOLUTION_8, True) # 8 micro-steps per macro-step (interpolated between each of the 200 macro-steps per rev)
        ssa.set_max_velocity(2000) # Velocity 2,000 steps/s (adjust this to get 1 rev per minute)
        # Slow ramping (500 steps/s^2),
        # Fast ramping (20,000 steps/s^2)
        ssa.set_speed_ramping(5000, 5000)
        # motor control        
        if pumpA1[i]=="F":
            logLines = logLine(logLines,"starting PMP-A1 forward, iter "+str(i))
            ssa.enable() # Enable motor power
            #ssa.set_steps(8*200) # Drive steps forward to make 1 rev
            ssa.drive_forward()
        if pumpA1[i]=="R":
            logLines = logLine(logLines,"starting PMP-A1 reverse, iter "+str(i))
            ssa.enable() # Enable motor power
            ssa.drive_backward()
        if pumpA1[i]=="N":
            logLines = logLine(logLines,"turning PMP-A1 off, iter "+str(i))
            ssa.stop() # Request motor stop
            ssa.set_speed_ramping(500, 5000) # Fast deacceleration (5000 steps/s^2) for stopping
            time.sleep(0.4) # Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
            ssa.disable() # Disable motor power

        ssc.set_motor_current(800) # 800 mA
        ssc.set_step_configuration(ssa.STEP_RESOLUTION_8, True) # 8 micro-steps per macro-step (interpolated between each of the 200 macro-steps per rev)
        ssc.set_max_velocity(2000) # Velocity 2,000 steps/s (adjust this to get 1 rev per minute)
        # Slow ramping (500 steps/s^2),
        # Fast ramping (20,000 steps/s^2)
        ssc.set_speed_ramping(5000, 5000)
        if pumpC1[i]=="F":
            logLines = logLine(logLines,"starting PMP-C1 forward, iter "+str(i))
            ssc.enable() # Enable motor power
            ssc.drive_forward()
        if pumpC1[i]=="R":
            logLines = logLine(logLines,"starting PMP-C1 reverse, iter "+str(i))
            ssc.enable() # Enable motor power
            ssc.drive_backward()
        if pumpC1[i]=="N":
            logLines = logLine(logLines,"turning PMP-C1 off, iter "+str(i))
            ssc.stop() # Request motor stop
            ssc.set_speed_ramping(500, 5000) # Fast deacceleration (5000 steps/s^2) for stopping
            time.sleep(0.4) # Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
            ssc.disable() # Disable motor power
        
        # define sampling rates for Tinkerforge peripherals
        logLines = logLine(logLines,"dt= " + str(dt) + " sec")
        logLines = logLine(logLines,"number of samples= " + str(n))   
        setSamplingRate()
        
        logLines = logLine(logLines,"experiment running...")
        # register callbacks 
        vc.register_callback(vc.CALLBACK_CURRENT, cb_current)
        vc.register_callback(vc.CALLBACK_VOLTAGE, cb_voltage)
        #tca.register_callback(tca.CALLBACK_TEMPERATURE, cb_tca)
        #tcc.register_callback(tcc.CALLBACK_TEMPERATURE, cb_tcc)
        kc.register_callback(kc.CALLBACK_VOLTAGE, cb_kc)
        ka.register_callback(ka.CALLBACK_VOLTAGE, cb_ka)
        rtda1.register_callback(rtda1.CALLBACK_TEMPERATURE, cb_rtda1)
        rtdc1.register_callback(rtdc1.CALLBACK_TEMPERATURE, cb_rtdc1)
        lca1.register_callback(lca1.CALLBACK_WEIGHT, cb_lca1)
        lca2.register_callback(lca2.CALLBACK_WEIGHT, cb_lca2)
        #lca3.register_callback(lca3.CALLBACK_WEIGHT, cb_lca3)
                
        # set period for current callback to 1s (1000ms) without a threshold
        vc.set_current_callback_configuration(1000.0*dt, False, "x", 0, 0)
        vc.set_voltage_callback_configuration(1000.0*dt, False, "x", 0, 0)
        #tca.set_temperature_callback_configuration(1000.0*dt, False, "x", 0, 0)
        #tcc.set_temperature_callback_configuration(1000.0*dt, False, "x", 0, 0)
        kc.set_voltage_callback_configuration(1000.0*dt, False, "x", 0, 0)
        ka.set_voltage_callback_configuration(1000.0*dt, False, "x", 0, 0)
        rtda1.set_temperature_callback_configuration(1000.0*dt, False, "x", 0, 0)
        rtdc1.set_temperature_callback_configuration(1000.0*dt, False, "x", 0, 0)
        lca1.set_weight_callback_configuration(1000.0*dt, False, "x", 0, 0)
        lca2.set_weight_callback_configuration(1000.0*dt, False, "x", 0, 0)
        #lca3.set_weight_callback_configuration(1000.0*dt, False, "x", 0, 0)

        # run experiment (threads put to sleep inside digilent_Aout())
        digilent_Aout(expList[i], hzFreq[i], cSamples[i], amp[i], offset[i], repeatCount[i], waitTime[i])

        # ensure experiment stops by sending a null experiment 
        digilent_Aout(filepathN, 1, 4096, 0, 0, 0, 0)
        
        # stop pumps
        ssc.stop() # Request motor stop
        ssc.set_speed_ramping(500, 5000) # Fast deacceleration (5000 steps/s^2) for stopping
        time.sleep(0.4) # Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
        ssc.disable() # Disable motor power
            
        ssa.stop() # Request motor stop
        ssa.set_speed_ramping(500, 5000) # Fast deacceleration (5000 steps/s^2) for stopping
        time.sleep(0.4) # Wait for motor to actually stop: max velocity (2000 steps/s) / decceleration (5000 steps/s^2) = 0.4 s
        ssa.disable() # Disable motor power
        
        # close all valves
        iqr14.set_value([False, False, False, False])
        iqr58.set_value([False, False, False, False])

        # Disconnect from Tinkerforge
        ipcon.disconnect()
        logLines = logLine(logLines,"experiment complete")
        
        logLines = logLine(logLines,"processing data...")
        # subtract initial time from time array and convert to seconds 
        iTime = (iTime - iTime[0])*1e-9
        eTime = (eTime - eTime[0])*1e-9
        #tcaTime = (tcaTime - tcaTime[0])*1e-9
        #tccTime = (tccTime - tccTime[0])*1e-9
        kcTime = (kcTime - kcTime[0])*1e-9
        kaTime = (kaTime - kaTime[0])*1e-9
        rtda1Time = (rtda1Time - rtda1Time[0])*1e-9
        rtdc1Time = (rtdc1Time - rtdc1Time[0])*1e-9
        lca1Time = (lca1Time - lca1Time[0])*1e-9
        lca2Time = (lca2Time - lca2Time[0])*1e-9
        #lca3Time = (lca3Time - lca3Time[0])*1e-9

        logLines = logLine(logLines, "----Pre-Processed Current-------")
        [dti, dti_mean, dti_std, dti_var, dti_min, dti_max] = sampleStats(iTime)
        logLines = logLine(logLines,"dt_mean= "+str(dti_mean))
        logLines = logLine(logLines,"dt_std= "+str(dti_std))
        logLines = logLine(logLines,"dt_max= "+str(dti_max))
        logLines = logLine(logLines,"dt_min= "+str(dti_min))
        logLines = logLine(logLines,"----Pre-Processed Voltage-------")
        [dte, dte_mean, dte_std, dte_var, dte_min, dte_max] = sampleStats(eTime)
        logLines = logLine(logLines,"dt_mean= "+str(dte_mean))
        logLines = logLine(logLines,"dt_std= "+str(dte_std))
        logLines = logLine(logLines,"dt_max= "+str(dte_max))
        logLines = logLine(logLines,"dt_min= "+str(dte_min))
        #logLines = logLine(logLines, "----Pre-Processed tca-------")
        #[dttca, dttca_mean, dttca_std, dttca_var, dttca_min, dttca_max] = sampleStats(tcaTime)
        #logLines = logLine(logLines,"dt_mean= "+str(dttca_mean))
        #logLines = logLine(logLines,"dt_std= "+str(dttca_std))
        #logLines = logLine(logLines,"dt_max= "+str(dttca_max))
        #logLines = logLine(logLines,"dt_min= "+str(dttca_min))
        #logLines = logLine(logLines,"----Pre-Processed tcc-------")
        #[dttcc, dttcc_mean, dttcc_std, dttcc_var, dttcc_min, dttcc_max] = sampleStats(tccTime)
        #logLines = logLine(logLines,"dt_mean= "+str(dttcc_mean))
        #logLines = logLine(logLines,"dt_std= "+str(dttcc_std))
        #logLines = logLine(logLines,"dt_max= "+str(dttcc_max))
        #logLines = logLine(logLines,"dt_min= "+str(dttcc_min))
        logLines = logLine(logLines,"----Pre-Processed Cathode Conductivity-------")
        [dtkc, dtkc_mean, dtkc_std, dtkc_var, dtkc_min, dtkc_max] = sampleStats(kcTime)
        logLines = logLine(logLines,"dt_mean= "+str(dtkc_mean))
        logLines = logLine(logLines,"dt_std= "+str(dtkc_std))
        logLines = logLine(logLines,"dt_max= "+str(dtkc_max))
        logLines = logLine(logLines,"dt_min= "+str(dtkc_min))
        logLines = logLine(logLines,"----Pre-Processed Anode Conductivity-------")
        [dtka, dtka_mean, dtka_std, dtka_var, dtka_min, dtka_max] = sampleStats(kaTime)
        logLines = logLine(logLines,"dt_mean= "+str(dtka_mean))
        logLines = logLine(logLines,"dt_std= "+str(dtka_std))
        logLines = logLine(logLines,"dt_max= "+str(dtka_max))
        logLines = logLine(logLines,"dt_min= "+str(dtka_min))
        logLines = logLine(logLines,"----Pre-Processed Anode RTD Temp-------")
        [dtrtda1, dtrtda1_mean, dtrtda1_std, dtrtda1_var, dtrtda1_min, dtrtda1_max] = sampleStats(rtda1Time)
        logLines = logLine(logLines,"dt_mean= "+str(dtrtda1_mean))
        logLines = logLine(logLines,"dt_std= "+str(dtrtda1_std))
        logLines = logLine(logLines,"dt_max= "+str(dtrtda1_max))
        logLines = logLine(logLines,"dt_min= "+str(dtrtda1_min))
        logLines = logLine(logLines,"----Pre-Processed Cathode RTD Temp-------")
        [dtrtdc1, dtrtdc1_mean, dtrtdc1_std, dtrtdc1_var, dtrtdc1_min, dtrtdc1_max] = sampleStats(rtdc1Time)
        logLines = logLine(logLines,"dt_mean= "+str(dtrtdc1_mean))
        logLines = logLine(logLines,"dt_std= "+str(dtrtdc1_std))
        logLines = logLine(logLines,"dt_max= "+str(dtrtdc1_max))
        logLines = logLine(logLines,"dt_min= "+str(dtrtdc1_min))
        logLines = logLine(logLines,"----Pre-Processed Milk Mass-------")
        [dtlca1, dtlca1_mean, dtlca1_std, dtcla1_var, dtlca1_min, dtlca1_max] = sampleStats(lca1Time)
        logLines = logLine(logLines,"dt_mean= "+str(dtlca1_mean))
        logLines = logLine(logLines,"dt_std= "+str(dtlca1_std))
        logLines = logLine(logLines,"dt_max= "+str(dtlca1_max))
        logLines = logLine(logLines,"dt_min= "+str(dtlca1_min))
        logLines = logLine(logLines,"----Pre-Processed DIW Mass-------")
        [dtlca2, dtlca2_mean, dtlca2_std, dtcla2_var, dtlca2_min, dtlca2_max] = sampleStats(lca2Time)
        logLines = logLine(logLines,"dt_mean= "+str(dtlca2_mean))
        logLines = logLine(logLines,"dt_std= "+str(dtlca2_std))
        logLines = logLine(logLines,"dt_max= "+str(dtlca2_max))
        logLines = logLine(logLines,"dt_min= "+str(dtlca2_min))
        #logLines = logLine(logLines,"----Pre-Processed Spare Mass-------")
        #[dtlca3, dtlca3_mean, dtlca3_std, dtcla3_var, dtlca3_min, dtlca3_max] = sampleStats(lca3Time)
        #logLines = logLine(logLines,"dt_mean= "+str(dtcla3_mean))
        #logLines = logLine(logLines,"dt_std= "+str(dtlca3_std))
        #logLines = logLine(logLines,"dt_max= "+str(dtlca3_max))
        #logLines = logLine(logLines,"dt_min= "+str(dtlca3_min))

        # resample data using linear interpolation method
        [iTime, iVal, i_n] = linResample(iTime, iVal, dt)
        [eTime, eVal, e_n] = linResample(eTime, eVal, dt)
        #[tcaTime, tcaVal, tca_n] = linResample(tcaTime, tcaVal, dt)
        #[tccTime, tccVal, tcc_n] = linResample(tccTime, tccVal, dt)
        [kcTime, kcVal, kc_n] = linResample(kcTime, kcVal, dt)
        [kaTime, kaVal, ka_n] = linResample(kaTime, kaVal, dt)
        [rtda1Time, rtda1Val, rtda1_n] = linResample(rtda1Time, rtda1Val, dt)
        [rtdc1Time, rtdc1Val, rtdc1_n] = linResample(rtdc1Time, rtdc1Val, dt)
        [lca1Time, lca1Val, lca1_n] = linResample(lca1Time, lca1Val, dt)
        [lca2Time, lca2Val, lca2_n] = linResample(lca2Time, lca2Val, dt)
        #[lca3Time, lca3Val, lca3_n] = linResample(lca3Time, lca3Val, dt)
        logLines = logLine(logLines,"data processing complete")
    
        logLines = logLine(logLines,"writing to csv...")
        # export data to csv file
        cnti = writeCSV(directory+'/data/data_'+str(i)+'_i.csv',iTime, iVal, 0) 
        cnte = writeCSV(directory+'/data/data_'+str(i)+'_e.csv',eTime, eVal, 0)
        #cnttca = writeCSV(directory+'/data/data_'+str(i)+'_tca.csv',tcaTime, tcaVal, 0)
        #cnttcc = writeCSV(directory+'/data/data_'+str(i)+'_tcc.csv',tccTime, tccVal, 0)
        cntkc = writeCSV(directory+'/data/data_'+str(i)+'_kc.csv',kcTime, kcVal, 0)
        cntkc = writeCSV(directory+'/data/data_'+str(i)+'_ka.csv',kaTime, kaVal, 0)
        cntrtda1 = writeCSV(directory+'/data/data_'+str(i)+'_rtda1.csv',rtda1Time, rtda1Val, 0)
        cntrtdc1 = writeCSV(directory+'/data/data_'+str(i)+'_rtdc1.csv',rtdc1Time, rtdc1Val, 0)
        cntlca1 = writeCSV(directory+'/data/data_'+str(i)+'_lca1.csv',lca1Time, lca1Val, 0)
        cntlca2 = writeCSV(directory+'/data/data_'+str(i)+'_lca2.csv',lca2Time, lca2Val, 0)
        #cntlca3 = writeCSV(directory+'/data/data_'+str(i)+'_lca3.csv',lca3Time, lca3Val, 0)
        logLines = logLine(logLines,"writing complete")
                
        logLines = logLine(logLines, "----Post-Processed Current-------")
        [dti, dti_mean, dti_std, dti_var, dti_min, dti_max] = sampleStats(iTime)
        logLines = logLine(logLines,"dt_mean= "+str(dti_mean))
        logLines = logLine(logLines,"dt_std= "+str(dti_std))
        logLines = logLine(logLines,"dt_max= "+str(dti_max))
        logLines = logLine(logLines,"dt_min= "+str(dti_min))
        logLines = logLine(logLines,"----Post-Processed Voltage-------")
        [dte, dte_mean, dte_std, dte_var, dte_min, dte_max] = sampleStats(eTime)
        logLines = logLine(logLines,"dt_mean= "+str(dte_mean))
        logLines = logLine(logLines,"dt_std= "+str(dte_std))
        logLines = logLine(logLines,"dt_max= "+str(dte_max))
        logLines = logLine(logLines,"dt_min= "+str(dte_min))
        #logLines = logLine(logLines, "----Post-Processed tca-------")
        #[dttca, dttca_mean, dttca_std, dttca_var, dttca_min, dttca_max] = sampleStats(tcaTime)
        #logLines = logLine(logLines,"dt_mean= "+str(dttca_mean))
        #logLines = logLine(logLines,"dt_std= "+str(dttca_std))
        #logLines = logLine(logLines,"dt_max= "+str(dttca_max))
        #logLines = logLine(logLines,"dt_min= "+str(dttca_min))
        #logLines = logLine(logLines, "----Post-Processed tcc-------")
        #[dttcc, dttcc_mean, dttcc_std, dttcc_var, dttcc_min, dttcc_max] = sampleStats(tccTime)
        #logLines = logLine(logLines,"dt_mean= "+str(dttcc_mean))
        #logLines = logLine(logLines,"dt_std= "+str(dttcc_std))
        #logLines = logLine(logLines,"dt_max= "+str(dttcc_max))
        #logLines = logLine(logLines,"dt_min= "+str(dttcc_min))
        logLines = logLine(logLines,"----Post-Processed Cathode Conductivity-------")
        [dtkc, dtkc_mean, dtkc_std, dtkc_var, dtkc_min, dtkc_max] = sampleStats(kcTime)
        logLines = logLine(logLines,"dt_mean= "+str(dtkc_mean))
        logLines = logLine(logLines,"dt_std= "+str(dtkc_std))
        logLines = logLine(logLines,"dt_max= "+str(dtkc_max))
        logLines = logLine(logLines,"dt_min= "+str(dtkc_min))
        logLines = logLine(logLines,"----Post-Processed Anode Conductivity-------")
        [dtka, dtka_mean, dtka_std, dtka_var, dtka_min, dtka_max] = sampleStats(kaTime)
        logLines = logLine(logLines,"dt_mean= "+str(dtka_mean))
        logLines = logLine(logLines,"dt_std= "+str(dtka_std))
        logLines = logLine(logLines,"dt_max= "+str(dtka_max))
        logLines = logLine(logLines,"dt_min= "+str(dtka_min))
        logLines = logLine(logLines,"----Post-Processed Anode RTD Temp-------")
        [dtrtda1, dtrtda1_mean, dtrtda1_std, dtrtda1_var, dtrtda1_min, dtrtda1_max] = sampleStats(rtda1Time)
        logLines = logLine(logLines,"dt_mean= "+str(dtrtda1_mean))
        logLines = logLine(logLines,"dt_std= "+str(dtrtda1_std))
        logLines = logLine(logLines,"dt_max= "+str(dtrtda1_max))
        logLines = logLine(logLines,"dt_min= "+str(dtrtda1_min))
        logLines = logLine(logLines,"----Post-Processed Cathode RTD Temp-------")
        [dtrtdc1, dtrtdc1_mean, dtrtdc1_std, dtrtdc1_var, dtrtdc1_min, dtrtdc1_max] = sampleStats(rtdc1Time)
        logLines = logLine(logLines,"dt_mean= "+str(dtrtdc1_mean))
        logLines = logLine(logLines,"dt_std= "+str(dtrtdc1_std))
        logLines = logLine(logLines,"dt_max= "+str(dtrtdc1_max))
        logLines = logLine(logLines,"dt_min= "+str(dtrtdc1_min))
        logLines = logLine(logLines,"----Post-Processed Milk Mass-------")
        [dtlca1, dtlca1_mean, dtlca1_std, dtcla1_var, dtlca1_min, dtlca1_max] = sampleStats(lca1Time)
        logLines = logLine(logLines,"dt_mean= "+str(dtlca1_mean))
        logLines = logLine(logLines,"dt_std= "+str(dtlca1_std))
        logLines = logLine(logLines,"dt_max= "+str(dtlca1_max))
        logLines = logLine(logLines,"dt_min= "+str(dtlca1_min))
        logLines = logLine(logLines,"----Post-Processed DIW Mass-------")
        [dtlca2, dtlca2_mean, dtlca2_std, dtcla2_var, dtlca2_min, dtlca2_max] = sampleStats(lca2Time)
        logLines = logLine(logLines,"dt_mean= "+str(dtlca2_mean))
        logLines = logLine(logLines,"dt_std= "+str(dtlca2_std))
        logLines = logLine(logLines,"dt_max= "+str(dtlca2_max))
        logLines = logLine(logLines,"dt_min= "+str(dtlca2_min))
        #logLines = logLine(logLines,"----Post-Processed Spare Mass-------")
        #[dtlca3, dtlca3_mean, dtlca3_std, dtcla3_var, dtlca3_min, dtlca3_max] = sampleStats(lca3Time)
        #logLines = logLine(logLines,"dt_mean= "+str(dtcla3_mean))
        #logLines = logLine(logLines,"dt_std= "+str(dtlca3_std))
        #logLines = logLine(logLines,"dt_max= "+str(dtlca3_max))
        #logLines = logLine(logLines,"dt_min= "+str(dtlca3_min))

        logLines = logLine(logLines,"%%%%%%%%%% end iteration "+str(i)+" %%%%%%%%%%")
        
        i+=1
    
    # Once all experiments complete, write to log file for later review
    #log_lines = ['', 'Append text files', 'The End']
    logLines = logLine(logLines,"End of Experiment")
    logLines = logLine(logLines,"writing log file")
    with open('log_DAQ.txt', 'w') as f:
        f.writelines('\n'.join(logLines))
exit()
    
    