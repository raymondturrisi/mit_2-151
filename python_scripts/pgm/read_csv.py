def read_csv_file(file_path, headed = True):
  # read data from file
  fh = open(file_path, "r")
  header = fh.readline().strip().split(',') if headed else None
  data = fh.readlines()
  fh.close()
  # remove newline and split each line using comma as delimiter
  for i, ln in enumerate(data):
    data[i] = list(map(lambda x: float(x), ln.strip().split(',')))
  return (header, data) if headed else data

def main():
  header, data = read_csv_file("test.csv")
  print(header, '\n\n', data)

if __name__ == "__main__":
  main()
