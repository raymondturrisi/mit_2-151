import csv
import os
import numpy as np

def readCSV(filename, startIndex):
    # reads time and measured value data from csv filename 

    # Returns:
    # time and value arrays
    directory = os.path.dirname(filename)
    if not os.path.exists(directory):
       return ["file does not exist"]

    # import data from file
    file = open(filename) 
    csvreader = csv.reader(file)
    data = []
    for row in csvreader:
            data.append(row)
    N = len(data)
    file.close()
    
    # create empty arrays for storage
    t=np.empty(N)
    x=np.empty(N)
    dt=np.empty(N)
    
    # store variables in arrays
    i=0
    while i<N:
        t[i]=float(data[i][0]) # timestamp
        x[i]=float(data[i][1]) # measurement
        if i>0:
            dt[i]=t[i]-t[i-1] # intersample times
        i=i+1

    return [t, x, N]


