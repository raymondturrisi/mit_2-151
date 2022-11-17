import numpy as np 

# resample data by interpolating between points
# this version creates a time vector of the same length as t with dt intervals
# note that the last sample is extrapolated

def linResample2(t, y, dt):
    # t is the original time array 
    # y is the original sampled signal
    # dt is the desired intersample time
    
    # Returns: 
    # ts resampled time array 
    # ys resampled signal
    # N number of resampled data poitns

    # create a uniformly spaced time array ts
    #t_end = t[-1] # time of last sample
    #N_raw = t_end/dt # number of evenly spaced samples form zero to end time
    #N = np.trunc(N_raw) # round down to nearst integer sample
    #N = N.astype(int) # cast float as integer
    N = len(t)
    #print("debug len N: "+str(N)) # debug
    t_end = N*dt
    ts = np.linspace(0,t_end, N) # time array with N evenly spaced times (dt) from 0 to end time
    #print("debug len ts: "+str(len(ts))) # debug
    ys = np.zeros(N) # create empty array for new values
    
    # loop through data to interpolate new points at uniform intersample times
    i=1
    while i < N:
        # extract time and value before and after 
        y_before = y[i-1]
        y_after = y[i]
        t_before = t[i-1]
        t_after = t[i]
        t_now = ts[i-1]
        # interpolate between y values
        ys[i-1] = y_before+((y_after-y_before)*((t_now-t_before)/(t_after-t_before)))

        i+=1

        # extrapolate last sample
        if i == N:
            t_now = ts[i-1] # extrapolated time beyond measured data
            ys[i-1] = y_after+((y_after-y_before)*((t_now-t_after)/(t_after-t_before)))

    return [ts, ys, N]