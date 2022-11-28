"""
    McKibben Actuator MICA Setup
    Author:  PGM
    Created:  2022-08-19
   
    Modified
    19 Aug 2022 - added calibration 
    23 Aug 2022 - added static test code
    02 Sep 2022 - modified test to be able to perform multiple loops and log to new files
    06 Sep 2022 - added logging to single file
    07 Sep 2022 - added energy monitor sensor
    07 Oct 2022 - added optional ouptut to 3/2 valve for electrolyzer test
    02 Nov 2022 - Removed electrolyzer hardware (3/2 valve and energy monitor)
"""

################### BEGIN USER CONFIG ###################
# calibration settings
doCalibration = False # perform calibration 

# static test settings
doStaticTest = False # perform the static test after calibration
numSteps = 20 # number of equally spaced steps to take for static testing
statTime = 50 # total time for static test experiment 

# dynamic test settings
doDynamicTest = True # perform the dynamic test after calibration
dynTime = 30
isRamp = False
isStep = True
sigStart = 12000 # start output value
sigStop = 16000 # stop output value

# general settings
interPause = 5 #160 # secs between experiments
iter = 1 # number of times to perform the experiment 

dt= 0.05 # change this to desired intersample interval for dynamic testing. e.g., 1.0 s

longDelay = 5 # seconds
shortDelay = 2 # seconds

################### END USER CONFIG ###################

# hardware addresses
HOST = "localhost"
PORT = 4223

UID_py1 = "GZM" # peizo output
UID_k1 = "Kr4" # relay output
UID_px = "Hfe" # pressure sensors (supply & controlled)
UID_zx1 = "Ji8" # position sensor
UID_lc1 = "S1C" # load cell
UID_master1 = "6EG9ps"
UID_master2 = "6s6yLQ"

# import required packages
import numpy as np 
import os
import time, datetime
from linResample import *
from sampleStats import *
from writeCSV import *
from writeCSV_multi import *
from readCSV import *
from logLine import *
import math 
# get TinkerForge components
from tinkerforge.ip_connection import IPConnection
from tinkerforge.brick_master import BrickMaster
# piezo Output 
from tinkerforge.bricklet_industrial_analog_out_v2 import BrickletIndustrialAnalogOutV2
# loadcell
from tinkerforge.bricklet_load_cell_v2 import BrickletLoadCellV2
# pressure sensors & Position sensor
from tinkerforge.bricklet_industrial_dual_0_20ma_v2 import BrickletIndustrialDual020mAV2
# relay output
from tinkerforge.bricklet_industrial_dual_relay import BrickletIndustrialDualRelay

directory = os.path.abspath(os.path.join(os.path.curdir)) # working directory root

def numSamples(testTime, deltaT):
    # caclulate the number of samples based on length of signal 
    T = np.ceil(testTime)
    T = T.astype(int)
    n = T/deltaT
    n=n.astype(int) # number of samples for tinkerforge
    return n

logLines = [] # list for logging and printing to console
# define number of samples in experiment based on time and dt

if doCalibration:
    n = numSamples(statTime, dt) # calculate number of samples
    selTest = "cal"
    logLines = logLine(logLines,"test selected: "+str(selTest))
    logLines = logLine(logLines,"statTime = "+str(statTime)+" sec")
if doStaticTest:
    n = numSamples(statTime, dt) # calculate number of samples
    selTest = "stat"
    logLines = logLine(logLines,"test selected: "+str(selTest))
    logLines = logLine(logLines,"statTime = "+str(statTime)+" sec")
if doDynamicTest:
    n = numSamples(dynTime, dt) # calculate number of samples
    selTest = "dyn"
    logLines = logLine(logLines,"test selected: "+str(selTest))
    logLines = logLine(logLines,"dynTime = "+str(dynTime)+" sec")

logLines = logLine(logLines,"dt= " + str(dt) + " sec")
logLines = logLine(logLines,"number of samples= " + str(n))   

# conversion constants to engineering units
# INSERT HERE

# define callback function for pressure measurements (this callback also contains control updates to py1)
def cb_px(channel, current):

    global epochTime

    global px1Val
    global px1Time
    global px1Index

    global px2Val
    global px2Time
    global px2Index 

    global py1Time
    global py1Val
    global py1Index

    global k1Val
    global k1Time
    global k1Index

    global n
    global sigOut

    # read supply pressure
    if px1Index < n and channel == 1:
        
        # record epoch time for comparison to flow insturment data
        epochTime[py1Index] = time.time()
        # perform control on peizo
        py1Time[py1Index] = time.perf_counter_ns()
        py1Val[py1Index] = int(sigOut[py1Index])
        py1.set_current(py1Val[py1Index]) # WRITE TO PEIZO HERE
        #print("py1 out = "+str(py1Val[py1Index])) # debugging
        
        # log relay state
        k1Time[k1Index] = time.perf_counter_ns()
        holder = k1.get_value()
        k1Val[k1Index] = holder[0]

        #print("k1 = "+str(k1Val[k1Index])) # debugging

        # log supply pressure
        px1Time[px1Index] = time.perf_counter_ns()
        px1Val[px1Index] = current/1000000.0 # mA

        # increment the index to keep track of number of samples collected for post processing
        py1Index = py1Index + 1
        k1Index = k1Index + 1
        px1Index = px1Index + 1
        
    if px2Index < n and channel == 0:
        #print("inside channel 2") #debugging
        px2Time[px2Index] = time.perf_counter_ns()
        px2Val[px2Index] = current/1000000.0 # mA

        # increment the index to keep track of number of samples collected for post processing
        px2Index = px2Index + 1 

# define callback function for position - ZX1
def cb_zx1(channel, current):
    global zx1Val
    global zx1Time
    global zx1Index
    global n
    
    if zx1Index < n and channel == 1:
        zx1Time[zx1Index] = time.perf_counter_ns()
        zx1Val[zx1Index] = current/1000000.0 # mA

        # increment the index to keep track of number of samples collected for post processing
        zx1Index = zx1Index + 1 

# define callback function for load cell - LC1
def cb_lc1(weight):
    global lc1Val
    global lc1Time
    global lc1Index
    global n
    
    if lc1Index < n:
        lc1Time[lc1Index] = time.perf_counter_ns()
        lc1Val[lc1Index] = weight # g

        # increment the index to keep track of number of samples collected for post processing
        lc1Index = lc1Index + 1 

# routine to calibrate position and force (load cell)        
def calPosition():
    
    global logLines

    # initialize piezo regulator
    logLines = logLine(logLines,"---------- starting calibration ----------")
    py1.set_current(4000)
    py1.set_enabled(True)
    py1_now = py1.get_current()
    logLines = logLine(logLines,"piezo current = "+str(py1_now/1000.0)+" mA")
    # unlock pneumatic brakes
    logLines = logLine(logLines,"unlock pneumatic brakes")
    k1.set_value(True, False)
    holder = k1.get_value()
    logLine(logLines, "k1[0] = "+str(holder[0])+", k1[1] = "+str(holder[1]))

    time.sleep(shortDelay)   
    
    # prompt user to move shafts up
    proceed = False
    while not proceed:
        userinput = input("move shafts up, then press Y + Enter:")
        print("key pressed:"+ userinput)
        if userinput == "Y":
            logLines = logLine(logLines,"lock pneumatic brakes")
            k1.set_value(False, False)
            time.sleep(2)
            proceed = True
    
    logLines = logLine(logLines,"tare load cell with shafts out of the way")
    lc1.tare()
    # show unweighted mass (without shafts)
    logLines = logLine(logLines,"post-tare mass = "+str(lc1.get_weight())+" g")

    # record initial position 
    zx1MinCal = zx1.get_current(1)
    logLines = logLine(logLines,"zx1MinCal = "+str(zx1MinCal/1000000.0)+" mA")
    time.sleep(shortDelay)
    # peizo to 100% of full
    logLines = logLine(logLines,"set piezo to 100 pct of full")
    py1.set_current(20000)
    py1_now = py1.get_current()
    logLines = logLine(logLines,"piezo current = "+str(py1_now/1000.0)+" mA")
    logLines = logLine(logLines,"hold "+str(longDelay)+"...")
    time.sleep(longDelay) # allow time for dynamics to settle
    # record final position 
    zx1MaxCal = zx1.get_current(1) 
    logLines = logLine(logLines,"zx1MaxCal = "+str(zx1MaxCal/1000000.0)+" mA")
    time.sleep(shortDelay)
    # unlock pneumatic brakes
    logLines = logLine(logLines,"unlock pneumatic brakes")
    k1.set_value(True, False)
    time.sleep(shortDelay) 
    # piezo to 20% of full 
    logLines = logLine(logLines,"set piezo to 25 pct of full")
    py1.set_current(8000)
    py1_now = py1.get_current()
    logLines = logLine(logLines,"piezo current = "+str(py1_now/1000.0)+" mA")
    time.sleep(shortDelay) # allow time for dynamics to settle
    # lock pneumatic brakes
    logLines = logLine(logLines,"lock pneumatic brakes")
    k1.set_value(False, False)
    time.sleep(shortDelay)
    # piezo back to 0% of full
    logLines = logLine(logLines,"return piezo to 0 pct of full")
    py1.set_current(4000)
    py1_now = py1.get_current()
    logLines = logLine(logLines,"piezo current = "+str(py1_now/1000.0)+" mA")
    logLines = logLine(logLines,"hold "+str(longDelay)+" sec...")
    time.sleep(longDelay) # allow time for dynamics to settle
    # record unweighted mass (without shafts)
    lc1MinCal = lc1.get_weight()
    logLines = logLine(logLines,"unweighted mass = "+str(lc1MinCal)+" g")
    # unlock pneumatic brakes
    logLines = logLine(logLines,"unlock pneumatic brakes")
    k1.set_value(True, False)
    logLines = logLine(logLines,"hold "+str(longDelay)+" sec...")
    time.sleep(longDelay)
    # record weighted mass (with shafts)
    lc1MaxCal = lc1.get_weight()
    logLines = logLine(logLines,"weighted mass = "+str(lc1MaxCal)+" g")
    time.sleep(shortDelay)
    # lock pneumatic brakes
    logLines = logLine(logLines,"lock pneumatic brakes")
    k1.set_value(False, False)
    time.sleep(shortDelay)
    
    logLines = logLine(logLines,"---------- calibration complete ----------")

    return [zx1MinCal, zx1MaxCal, lc1MinCal, lc1MaxCal]

def linearOutput(start, stop, expTime, deltaT):
    # generate linear output signal from start to stop over length of time
    numSamples = expTime/deltaT # number of samples scales with intersample time
    interval = (stop-start)/numSamples # interval between start and stop
    linOut = np.arange(start, stop+interval, interval, dtype=int) # array of evenly spaced steps
    linOut[-1] = stop # force last value to stop value to ensure max output reached
    return linOut

def stepOutput(start, step, expTime, deltaT):
    # generate linear output signal from start to stop over length of time
    duration = expTime/3.0
    samples = math.ceil(duration/deltaT)
    init_vals = np.ones(samples)*start
    step_vals = np.ones(samples)*step
    stepOut = np.int32(np.concatenate((init_vals, step_vals, init_vals), axis=0))
    return stepOut

def staticTest(numSteps):

    global logLines
    global shortDelay
    global n
    global statTime
    global dt

    py1V = [] # piezo otput 
    k1V = []  # relay output 
    px1V = [] # supply pressure measurements
    px2V = [] # regulated pressure measurements
    zx1V = [] # position measurements
    lc1V = [] # force measurements 

    py1T = [] 
    k1T = []
    px1T = [] 
    px2T = [] 
    zx1T = [] 
    lc1T = [] 
    epochT = []

    interval = 16000/numSteps # interval size between 4mA and 20mA

    #CONFIGURE OUTPUT
    steps = np.arange(sigStart, sigStop+interval, interval, dtype=int) # array of evenly spaced steps
    #sigOut = linearOutput(4000, 20000, statTime, dt) # generate linear output signal, time and dt determine num samples for static test
    sigOut = stepOutput(sigStart, sigStop, statTime, dt) # generate linear output signal, time and dt determine num samples for static test

    # send sigOut signal to piezo, when threshold value reached, lock the shafts and take readings
    for iStep in steps:
        logLines = logLine(logLines,"iStep value = "+str(iStep))
        k1.set_value(True, False) # unlock pneumatic brakes
        time.sleep(shortDelay)
    
        i=0
        for iSig in sigOut: 
           
            if iSig >= iStep:
                k1.set_value(False, False) # lock pneumatic brakes
                time.sleep((iStep/40000)**2)  # allow time for mechanical dynamics to play out, more time for larger stroke value
                
                epochT = np.append(epochT, time.time()) # log system time since epoch
                py1T = np.append(py1T, time.perf_counter_ns()) 
                py1V = np.append(py1V, py1.get_current()) # log piezo output
                k1T = np.append(k1T, time.perf_counter_ns()) 
                holder = k1.get_value()
                k1V = np.append(k1V, holder[0]) # log relay output
                px1T = np.append(px1T, time.perf_counter_ns()) 
                px1V = np.append(px1V, px.get_current(1)) # log supply pressure
                px2T = np.append(px2T, time.perf_counter_ns()) 
                px2V = np.append(px2V, px.get_current(0)) # log regulated pressure
                zx1T = np.append(zx1T, time.perf_counter_ns()) 
                zx1V = np.append(zx1V, zx1.get_current(1)) # log position 
                lc1T = np.append(lc1T, time.perf_counter_ns())
                lc1V = np.append(lc1V, lc1.get_weight()) # log force
                
                py1.set_current(iSig) # set new value of peizo output
                
            else:
                k1.set_value(True, False) # ensure pneumatic brakes unlocked
                py1.set_current(iSig) # set new value of peizo output
                time.sleep(dt) # wait for approx dt time, not critical for static testing
            i+=1
        py1.set_current(4000) # bleed all air before moving to next step in test
        time.sleep(0.2) 
        k1.set_value(True, False) # unlock pneumatic brakes
    
    return [epochT, steps, py1T, py1V, k1T, k1V, px1T, px1V, px2T, px2V, zx1T, zx1V, lc1T, lc1V]

def zx1EU(zx1_nA):
        
    m = -100.0/16000000.0 # slope of nA to mm conversion
    b = 125 # offset of nA to mm conversion
    zx1_mm = m*zx1_nA+b # linear conversion for nA to mm

    return[zx1_mm]

def setSamplingRate():

    global logLines
    logLines = logLine(logLines,"-------begin tinkerforge configuration-------")
    px.set_sample_rate(2) # 2=15 samples per second, 16 bit resolution
    logLines = logLine(logLines,"PX1 & PX2 sampling rate config = "+str(px.get_sample_rate()))
    zx1.set_sample_rate(2) # 2=15 samples per second, 16 bit resolution
    logLines = logLine(logLines,"ZX1 sampling rate config = "+str(zx1.get_sample_rate()))
    lc1.set_moving_average(1) # 1=averaging off
    logLines = logLine(logLines,"LC1 moving average (1=off) = "+str(lc1.get_moving_average()))
    lc1.set_configuration(0, 0), # RATE_10HZ = 0, GAIN_128X = 0 (2mV/V)
    logLines = logLine(logLines,"LC1 config (RATE_10HZ = 0, GAIN_128X = 0 (2mV/V))= "+str(lc1.get_configuration()))
    logLines = logLine(logLines,"-------end tinkerforge configuration-------")

if __name__ == "__main__":

    i = 0
    # number of iterations defined in header
    
    while i<iter:

        logLines = logLine(logLines,"%%%%%%%%%% start iteration "+str(i)+" %%%%%%%%%%")
        
        if i>0:
            logLines = logLine(logLines,"pause for "+str(interPause)+" sec before starting")
            time.sleep(interPause) # pause between experiments   

        logLines = logLine(logLines,"initialize measurement arrays")
        # initialize measurement arrays
        py1Val = np.zeros(n) # piezo output
        k1Val = np.zeros(n) # relay output
        px1Val = np.zeros(n) # supply pressure measurements
        px2Val = np.zeros(n) # regulated pressure measurements
        zx1Val = np.zeros(n) # position measurements
        lc1Val = np.zeros(n) # force measurements
       
        # timestamps
        epochTime = np.zeros(n)
        py1Time = np.zeros(n)
        k1Time = np.zeros(n)  
        px1Time = np.zeros(n) 
        px2Time = np.zeros(n) 
        zx1Time = np.zeros(n) 
        lc1Time = np.zeros(n) 

        # sample indexes
        py1Index = 0
        k1Index = 0
        px1Index = 0
        px2Index = 0
        zx1Index = 0
        lc1Index = 0
        
        # initialize calibration variables
        zx1MinCal = 0.0
        zx1MaxCal = 0.0
        lc1MinCal = 0.0
        lc1MaxCal = 0.0
                
        logLines = logLine(logLines,"connecting to tinkerforge hardware...")  
        ipcon = IPConnection() # Create IP connection
        
        py1 = BrickletIndustrialAnalogOutV2(UID_py1, ipcon) # peizo output
        k1 = BrickletIndustrialDualRelay(UID_k1, ipcon) # relay output
        px = BrickletIndustrialDual020mAV2(UID_px, ipcon) # pressure sensors
        zx1 = BrickletIndustrialDual020mAV2(UID_zx1, ipcon) # position sensor
        lc1 = BrickletLoadCellV2(UID_lc1, ipcon) # load cell 
        master1 = BrickMaster(UID_master1, ipcon) 
        master2 = BrickMaster(UID_master2, ipcon) 

        ipcon.connect(HOST, PORT) # Connect to brick
        # Don't use device before ipcon is connected
        time.sleep(0.2)   
        logLines = logLine(logLines,"done connecting")

        # show initial raw values
        logLines = logLine(logLines,"showing intial raw values")
        px1_now = px.get_current(1)
        logLines = logLine(logLines,"Pressure PX-1: "+str(px1_now/1000000.0)+" mA")
        px2_now = px.get_current(0)
        logLines = logLine(logLines,"Pressure PX-2: "+str(px2_now/1000000.0)+" mA")
        zx1_now = zx1.get_current(1)
        logLines = logLine(logLines,"Position ZX-1: "+str(zx1_now/1000000.0)+" mA")
        lc1_now = lc1.get_weight()
        logLines = logLine(logLines,"Load Cell LC-1: "+str(lc1_now)+" g")
        
        py1.set_enabled(True) # enable peizo output for following tests

        if doCalibration:
            [zx1MinCal, zx1MaxCal, lc1MinCal, lc1MaxCal] = calPosition()
        
            logLines = logLine(logLines,"zxMinCal = "+str(zx1MinCal/1000000.0)+" mA")
            logLines = logLine(logLines,"zxMaxCal = "+str(zx1MaxCal/1000000.0)+" mA")
            [zx1Min] = zx1EU(zx1MinCal)
            [zx1Max] = zx1EU(zx1MaxCal)
            logLines = logLine(logLines,"zxMin = "+str(zx1Min)+" mm")
            logLines = logLine(logLines,"zxMax = "+str(zx1Max)+" mm")
        
        if doStaticTest:
            logLines = logLine(logLines,"beginning static test...")
            setSamplingRate() # configuration parameters for tinkerforge peripherals
            [epochTime, steps, py1Time, py1Val, k1Time, k1Val, px1Time, px1Val, \
                px2Time, px2Val, zx1Time, zx1Val, lc1Time, lc1Val] = staticTest(numSteps) 
            k1.set_value(True, False) # unlock pneumatic brakes
            time.sleep(shortDelay)
            logLines = logLine(logLines,"static test steps = "+str(steps))
            logLines = logLine(logLines,"...static test complete")
            
        if doDynamicTest:

            setSamplingRate() # configuration parameters for tinkerforge peripherals
            
            logLines = logLine(logLines,"unlock pneumatic brakes")
            k1.set_value(True, False)

            # prompt user to move shafts up
            proceed = False
            while not proceed:
                userinput = input("move shafts up, then press Y + Enter:")
                print("key pressed:"+ userinput)
                if userinput == "Y":
                    logLines = logLine(logLines,"lock pneumatic brakes")
                    k1.set_value(False, False)
                    time.sleep(2)
                    proceed = True

            if isRamp:
                logLines = logLine(logLines,"dynamic ramp test selected")
                sigOut = linearOutput(sigStart, sigStop, dynTime, dt) # generate linear output signal
                print("sigOut[1:10] = "+str(sigOut[1:10]))

            if isStep:
                logLines = logLine(logLines,"dynamic step test selected")
                sigOut = stepOutput(sigStart, sigStop, dynTime, dt) # step response
                print("sigOut[1:10] = "+str(sigOut[1:10]))
            
            logLines = logLine(logLines,"dynamic test running...")
            # register callbacks 
            px.register_callback(px.CALLBACK_CURRENT, cb_px)
            zx1.register_callback(zx1.CALLBACK_CURRENT, cb_zx1)
            lc1.register_callback(lc1.CALLBACK_WEIGHT, cb_lc1)
    
            # set period for current callback to 1s*dt (1000ms*dt) without a threshold
            px.set_current_callback_configuration(1, 1000*dt, False, "x", 0, 0)
            px.set_current_callback_configuration(0, 1000*dt, False, "x", 0, 0)
            zx1.set_current_callback_configuration(1, 1000*dt, False, "x", 0, 0)
            lc1.set_weight_callback_configuration(1000*dt, False, "x", 0, 0)

            time.sleep(dt*n) # put threads to sleep, all updates are performed inside the callback functions
            time.sleep(shortDelay) # just in case
            
            # piezo back to 100% of full to engage brakes
            logLines = logLine(logLines,"sequence to unlock brakes")
            py1.set_current(20000)
            py1_now = py1.get_current()
            logLines = logLine(logLines,"piezo current = "+str(py1_now/1000.0)+" mA")
            logLines = logLine(logLines,"hold "+str(shortDelay)+" sec...")
            time.sleep(shortDelay) # allow time for dynamics to settle
            # unlock pneumatic brakes
            logLines = logLine(logLines,"unlock pneumatic brakes")
            k1.set_value(True, False)
            logLines = logLine(logLines,"hold "+str(shortDelay)+" sec...")
            time.sleep(shortDelay)
            # piezo back to 0% of full to lower shafts
            py1.set_current(4000)
            py1_now = py1.get_current()
            logLines = logLine(logLines,"piezo current = "+str(py1_now/1000.0)+" mA")
            logLines = logLine(logLines,"hold "+str(shortDelay)+" sec...")
            time.sleep(shortDelay)

            logLines = logLine(logLines,"...dynamic test complete")


        logLines = logLine(logLines,"lock pneumatic brakes")
        k1.set_value(False, False) # lock shafts
        logLines = logLine(logLines,"disable peizo output")
        py1.set_enabled(False) # disable piezo output
        # disconnect from host
        ipcon.disconnect()
        logLines = logLine(logLines,"experiment complete")
        
        # Post process and write data to file
        timestr = time.strftime("%Y%m%d-%H%M%S") # current time string for appending to filename
        logLines = logLine(logLines,"logging data at: "+str(timestr))
            
        if not doCalibration:
            if i == 0:
                # make new directory with current datetime
                mydirR = os.path.join(os.getcwd()+'/data', \
                    datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S_raw_'+str(selTest)))
                logLines = logLine(logLines,"creating new directory: "+str(mydirR))
                os.makedirs(mydirR)
            mydir = mydirR # for writing log file

            # write raw data files 
            logLines = logLine(logLines,"writing raw data to csv")
            cntet = writeCSV(mydirR+'/exp_'+str(i)+'_et.csv',py1Time, epochTime, 0) # only need to write epoch time to raw data 
            cntpy1 = writeCSV(mydirR+'/exp_'+str(i)+'_py1.csv',py1Time, py1Val, 0)
            cntk1 = writeCSV(mydirR+'/exp_'+str(i)+'_k1.csv',k1Time, k1Val, 0)
            cntpx1 = writeCSV(mydirR+'/exp_'+str(i)+'_px1.csv',px1Time, px1Val, 0)
            cntpx2 = writeCSV(mydirR+'/exp_'+str(i)+'_px2.csv',px2Time, px2Val, 0)
            cntzx1 = writeCSV(mydirR+'/exp_'+str(i)+'_zx1.csv',zx1Time, zx1Val, 0)
            cntlc1 = writeCSV(mydirR+'/exp_'+str(i)+'_lc1_.csv',lc1Time, lc1Val, 0)

        if doDynamicTest:
            
            # write data files
            if i == 0:
                # make new directory with current datetime
                mydirP = os.path.join(os.getcwd()+'/data', \
                    datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S_processed_'+str(selTest)))
                #mydir = os.path.join(os.getcwd(), datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S_processed'))
                logLines = logLine(logLines,"creating new directory: "+str(mydirP))
                os.makedirs(mydirP)
            mydir = mydirP # for writing log file

            logLines = logLine(logLines,"subtracting initial time value from each set")
            # subtract initial time from time array and convert from ns to seconds 
            py1Time = (py1Time - py1Time[0])*1e-9
            k1Time = (k1Time - k1Time[0])*1e-9
            px1Time = (px1Time - px1Time[0])*1e-9
            px2Time = (px2Time - px2Time[0])*1e-9
            zx1Time = (zx1Time - zx1Time[0])*1e-9
            lc1Time = (lc1Time - lc1Time[0])*1e-9
            
            logLines = logLine(logLines,"----Pre-Processed PY1-------")
            [dtpy1, dtpy1_mean, dtpy1_std, dtpy1_var, dtpy1_min, dtpy1_max] = sampleStats(py1Time)
            logLines = logLine(logLines,"dt_mean= "+str(dtpy1_mean))
            logLines = logLine(logLines,"dt_std= "+str(dtpy1_std))
            logLines = logLine(logLines,"dt_max= "+str(dtpy1_max))
            logLines = logLine(logLines,"dt_min= "+str(dtpy1_min))       
            logLines = logLine(logLines,"----Pre-Processed PX1-------")
            [dtpx1, dtpx1_mean, dtpx1_std, dtpx1_var, dtpx1_min, dtpx1_max] = sampleStats(px1Time)
            logLines = logLine(logLines,"dt_mean= "+str(dtpx1_mean))
            logLines = logLine(logLines,"dt_std= "+str(dtpx1_std))
            logLines = logLine(logLines,"dt_max= "+str(dtpx1_max))
            logLines = logLine(logLines,"dt_min= "+str(dtpx1_min))
            logLines = logLine(logLines,"----Pre-Processed PX2-------")
            [dtpx2, dtpx2_mean, dtpx2_std, dtpx2_var, dtpx2_min, dtpx2_max] = sampleStats(px2Time)
            logLines = logLine(logLines,"dt_mean= "+str(dtpx2_mean))
            logLines = logLine(logLines,"dt_std= "+str(dtpx2_std))
            logLines = logLine(logLines,"dt_max= "+str(dtpx2_max))
            logLines = logLine(logLines,"dt_min= "+str(dtpx2_min))
            logLines = logLine(logLines,"----Pre-Processed ZX1-------")
            [dtzx1, dtzx1_mean, dtzx1_std, dtzx1_var, dtzx1_min, dtzx1_max] = sampleStats(zx1Time)
            logLines = logLine(logLines,"dt_mean= "+str(dtzx1_mean))
            logLines = logLine(logLines,"dt_std= "+str(dtzx1_std))
            logLines = logLine(logLines,"dt_max= "+str(dtzx1_max))
            logLines = logLine(logLines,"dt_min= "+str(dtzx1_min))
            logLines = logLine(logLines,"----Pre-Processed LC1-------")
            [dtlc1, dtlc1_mean, dtlc1_std, dtcl1_var, dtlc1_min, dtlc1_max] = sampleStats(lc1Time)
            logLines = logLine(logLines,"dt_mean= "+str(dtlc1_mean))
            logLines = logLine(logLines,"dt_std= "+str(dtlc1_std))
            logLines = logLine(logLines,"dt_max= "+str(dtlc1_max))
            logLines = logLine(logLines,"dt_min= "+str(dtlc1_min))
            
            # resample data using linear interpolation method              
            logLines = logLine(logLines,"resampling data using linear interpolation")
            [py1Time, py1Val, py1_n] = linResample(py1Time, py1Val, dt)
            [k1Time, k1Val, k1_n] = linResample(k1Time, k1Val, dt)
            [px1Time, px1Val, px1_n] = linResample(px1Time, px1Val, dt)
            [px2Time, px2Val, px2_n] = linResample(px2Time, px2Val, dt)
            [zx1Time, zx1Val, zx1_n] = linResample(zx1Time, zx1Val, dt)
            [lc1Time, lc1Val, lc1_n] = linResample(lc1Time, lc1Val, dt)

            # cut off data sets to match the shortest data set
            allDataL = [py1_n, k1_n, px1_n, px2_n, zx1_n, lc1_n]
            cutLen = np.min(allDataL)
            cutInd = cutLen - 1
            
            py1Time=py1Time[0:cutInd]
            k1Time=k1Time[0:cutInd]
            px1Time=px1Time[0:cutInd]
            px2Time=px2Time[0:cutInd]
            zx1Time=zx1Time[0:cutInd]
            lc1Time=lc1Time[0:cutInd]
            
            py1Val=py1Val[0:cutInd]
            k1Val=k1Val[0:cutInd]
            px1Val=px1Val[0:cutInd]
            px2Val=px2Val[0:cutInd]
            zx1Val=zx1Val[0:cutInd]
            lc1Val=lc1Val[0:cutInd]

            logLines = logLine(logLines,"----Post-Processed PY1-------")
            [dtpy1, dtpy1_mean, dtpy1_std, dtpy1_var, dtpy1_min, dtpy1_max] = sampleStats(py1Time)
            logLines = logLine(logLines,"dt_mean= "+str(dtpy1_mean))
            logLines = logLine(logLines,"dt_std= "+str(dtpy1_std))
            logLines = logLine(logLines,"dt_max= "+str(dtpy1_max))
            logLines = logLine(logLines,"dt_min= "+str(dtpy1_min))
            logLines = logLine(logLines,"----Post-Processed PX1-------")
            [dtpx1, dtpx1_mean, dtpx1_std, dtpx1_var, dtpx1_min, dtpx1_max] = sampleStats(px1Time)
            logLines = logLine(logLines,"dt_mean= "+str(dtpx1_mean))
            logLines = logLine(logLines,"dt_std= "+str(dtpx1_std))
            logLines = logLine(logLines,"dt_max= "+str(dtpx1_max))
            logLines = logLine(logLines,"dt_min= "+str(dtpx1_min))
            logLines = logLine(logLines,"----Post-Processed PX2-------")
            [dtpx2, dtpx2_mean, dtpx2_std, dtpx2_var, dtpx2_min, dtpx2_max] = sampleStats(px2Time)
            logLines = logLine(logLines,"dt_mean= "+str(dtpx2_mean))
            logLines = logLine(logLines,"dt_std= "+str(dtpx2_std))
            logLines = logLine(logLines,"dt_max= "+str(dtpx2_max))
            logLines = logLine(logLines,"dt_min= "+str(dtpx2_min))
            logLines = logLine(logLines,"----Post-Processed ZX1-------")
            [dtzx1, dtzx1_mean, dtzx1_std, dtzx1_var, dtzx1_min, dtzx1_max] = sampleStats(zx1Time)
            logLines = logLine(logLines,"dt_mean= "+str(dtzx1_mean))
            logLines = logLine(logLines,"dt_std= "+str(dtzx1_std))
            logLines = logLine(logLines,"dt_max= "+str(dtzx1_max))
            logLines = logLine(logLines,"dt_min= "+str(dtzx1_min))
            logLines = logLine(logLines,"----Post-Processed LC1-------")
            [dtlc1, dtlc1_mean, dtlc1_std, dtcl1_var, dtlc1_min, dtlc1_max] = sampleStats(lc1Time)
            logLines = logLine(logLines,"dt_mean= "+str(dtlc1_mean))
            logLines = logLine(logLines,"dt_std= "+str(dtlc1_std))
            logLines = logLine(logLines,"dt_max= "+str(dtlc1_max))
            logLines = logLine(logLines,"dt_min= "+str(dtlc1_min))
            
            logLines = logLine(logLines,"writing processed data to csv")
            cntpy1 = writeCSV(mydirP+'/exp_'+str(i)+'_py1.csv',py1Time, py1Val, 0)
            cntk1 = writeCSV(mydirP+'/exp_'+str(i)+'_k1.csv',k1Time, k1Val, 0)
            cntpx1 = writeCSV(mydirP+'/exp_'+str(i)+'_px1.csv',px1Time, px1Val, 0)
            cntpx2 = writeCSV(mydirP+'/exp_'+str(i)+'_px2.csv',px2Time, px2Val, 0)
            cntzx1 = writeCSV(mydirP+'/exp_'+str(i)+'_zx1.csv',zx1Time, zx1Val, 0)
            cntlc1 = writeCSV(mydirP+'/exp_'+str(i)+'_lc1_.csv',lc1Time, lc1Val, 0) 
        
            
            logLines = logLine(logLines,"writing processed data to a compiled csv")
            hdr = ['py1Time', 'py1Val', 'k1Val', 'k1Val2', 'px1Val', 'px2Val', 'zx1Val', 'lc1Val']
            allVals = np.vstack((py1Time, py1Val, k1Val, px1Val, px2Val, zx1Val, lc1Val))
            #allVals = np.transpose(allVals) # do not take transpose 
            cntall = writeCSV_multi(mydirP+'/exp_'+str(i)+'_compiled.csv', hdr, allVals, 0)
                
        logLines = logLine(logLines,"%%%%%%%%%% end iteration "+str(i)+" %%%%%%%%%%")
        i+=1

    # Once all experiments complete, write to log file for later review
    logLines = logLine(logLines,"End of Experiment")
    logLines = logLine(logLines,"writing log file")
    with open(mydir+'/log_DAQ.txt', 'w') as f:
        f.writelines('\n'.join(logLines))

exit() 
