
import numpy as np 

# print time statistics 
def sampleStats(timeArr):
    # timeArr is an array of values

    # Returns:
    # statistics for intersample time dt
    
    # subtract initial time for dt calculation
    #timeVec = timeVec - timeVec[0]
    dt = np.empty(len(timeArr)-1)
    i=1 # skip index 0 since we are finding a dt 
    while i<=len(dt):
        dt[i-1] = timeArr[i]-timeArr[i-1]
        i=i+1
    # calculate stats for dt
    dt_mean = np.mean(dt)
    dt_std = np.std(dt)
    dt_var = np.var(dt)
    dt_min = np.min(dt)
    dt_max = np.max(dt)
    
    return [dt, dt_mean, dt_std, dt_var, dt_min, dt_max]