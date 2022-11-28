import csv
import os
import numpy as np

# write data to csv file
def check_dir(filename):
    directory = os.path.dirname(filename)
    if not os.path.exists(directory):
        os.makedirs(directory)

def writeCSV_multi(filename, hdr, val, startIndex):
    # writes time and val arrays to filename as csv

    # Returns:
    # number final row number
    
    check_dir(filename) # create file if it does not exist
    csv_file_x = open(filename, 'w', newline='') # Mac OS: csv_file_loadcell = open('loadcell.csv', 'w') 
    csv_writer_x = csv.writer(csv_file_x)
    
    #header first
    row_x=hdr
    csv_writer_x.writerow(row_x)

    # then data  
    row = 0 + startIndex
    s = np.shape(val) # [col][row]
    print('internal shape = '+str(s))
    while row < s[1]:
        row_x=val[:,row]
        csv_writer_x.writerow(row_x)
        row+=1

    csv_file_x.close() # close file

    return row

