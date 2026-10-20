import csv

def write_csv(rows, cols, output_file):
    """ Write a list of dictionaries to a CSV file. """
    with open(output_file, "w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=cols)
        writer.writeheader()
        writer.writerows(rows)

def read_csv(file_name):
    """ Read a CSV file and return its rows and columns """
    with open(file_name, "r", newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        rows = list(reader)
        return rows, reader.fieldnames
