import csv
import os

# write data to csv file
def check_dir(filename):
    directory = os.path.dirname(filename)
    if not os.path.exists(directory):
        os.makedirs(directory)

def writeCSV(filename, time, val, startIndex):
    # writes time and val arrays to filename as csv

    # Returns:
    # number of rows

    check_dir(filename) # create file if it does not exist

    # open file and define writer
    csv_file_x = open(filename, 'w', newline='') # Mac OS: csv_file_loadcell = open('loadcell.csv', 'w') 
    csv_writer_x = csv.writer(csv_file_x)
    # write data
    count = 0 + startIndex
    for t in time:
        row_x = [str(time[count]), str(val[count])]
        csv_writer_x.writerow(row_x)
        count+=1
    csv_file_x.close() # close file

    return count

