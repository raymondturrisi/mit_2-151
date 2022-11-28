import numpy as np 

# resample data by interpolating between points
# this version defines the last timestamp as the final time and creates a vector of evenly spaced (dt) times between

def linResample(t, y, dt):
    # t is the original time array 
    # y is the original sampled signal
    # dt is the desired intersample time
    
    # Returns: 
    # ts resampled time array 
    # ys resampled signal
    # N number of resampled data points

    # create a uniformly spaced time array ts
    t_end = t[-1] # time of last sample
    N_raw = t_end/dt # number of evenly spaced samples form zero to end time
    N = np.trunc(N_raw) # round down to nearst integer sample
    N = N.astype(int) # cast float as integer
    ts = np.linspace(0,t_end, N) # time array with N evenly spaced times (dt) from 0 to end time
    ys = np.zeros(N) # create empty array for new values
    
    # loop through data to interpolate new points at uniform intersample times
    for i in range(N):
        j=0
        while ts[i] > t[j]:
            j=j+1
        
        # extract time and value before and after 
        y_before = y[j-1]
        y_after = y[j]
        t_before = t[j-1]
        t_after = t[j]
        t_now = ts[i]
        # interpolate between y values
        ys[i] = y_before+((y_after-y_before)*((t_now-t_before)/(t_after-t_before)))
    
    return [ts, ys, N]