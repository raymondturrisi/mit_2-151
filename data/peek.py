#Quick script for previewing data - taken from the plim_analyses jupyter notebook
#Useage:
#$ python3 peek.py data_day/test_directory_exp
import os, sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

if len(sys.argv) != 2:
    print("Useage: \n\t$python3 peek.py data/data_day/data_dir")
    exit(1)


def plot_test(df):
    fig, axes = plt.subplots(5,1, sharex=True, figsize=(15,10))
    axes[0].plot(df["t"], df["y"],linestyle='None',marker='o',markersize=1)
    axes[0].plot(0, 0,linestyle='None',marker='o',markersize=1, color='r')
    axes[0].grid()
    axes[0].set_ylabel("Distance (mm)")

    axes[1].plot(df["t"], df["l"],linestyle='None',marker='o',markersize=1,color='r')
    axes[1].grid()
    axes[1].set_ylabel("Load (grams)")
    
    axes[2].plot(df["t"], df["p_o"],linestyle='None',marker='o',markersize=1)
    axes[2].grid()
    axes[2].plot(df["t"], df["p_p"],linestyle='None',marker='o',markersize=1)
    axes[2].set_ylabel("Pressure - Actuator \n(Bar)")
    #axes[2].set_ylim([-0.25, 6.25])

    axes[2].legend(["Pressure Out","Pressure Gauge"])

    axes[3].plot(df["t"], df["p_c"],linestyle='None',marker='o',markersize=1)
    axes[3].grid()
    #axes[2].set_ylim([-0.25, 6.25])
    axes[3].set_ylabel("Pressure - Comp. \n(Bar)")

    axes[4].plot(df["t"], df["v"],linestyle='None',marker='o',markersize=1)
    axes[4].grid()
    axes[4].set_ylabel("System (V)")
    plt.suptitle("Experiment")
    plt.show()


data = []
init_length = 300


dirname = sys.argv[1]
files = os.listdir(dirname)
for file in files:
    if ".dat" in file:
        fname = (f"{dirname}/{file}")
        print(file)
dat = pd.read_csv(fname)
dat["t"] = dat["T_(ms)"]/1000 #ms to s
#dat["y"] = 90.08745-((dat["V_dist_(mA)"]-4)/16)*101.60 #convert milliamp to millimeters
dat["y"] = (dat["V_dist_(mA)"]*(-100.0/16.0)+125-11.2625) #convert milliamp to millimeters
dat["strain"] = (init_length-dat["y"])/init_length
dat["p_c"] = (dat["Comp_P_(mA)"]-4)*0.75
dat["v"] = dat["Sys_Volts_(V)"]
dat["l"] = dat["LC_(grams)"]
dat["p_p"] = (dat["Piezo_P_(mA)"]-4)*0.75
dat["p_o"] = 6*(dat["Piezo_out_(mV)"]-4000)/16000 #converting milliamp output to expected bar output
dat.style.set_caption(fname.split('/')[-1].split('.')[0])

print(fname)

plot_test(dat)