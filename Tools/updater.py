import ast
import csv
import shutil
from pathlib import Path
from scanner import can_edit_value
from csv_manager import read_csv, write_csv

def update_config_parameter(file_name, location_row, new_value):
    """ When New Value column in csv file is changed, the Config file is also changed based on the location row """
    path = Path(file_name)
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=str(path))
    
    # check if there was a new_value inputted into the csv file
    if new_value == None:
        return False
    new_value = str(new_value).strip()
    if new_value == "":
        return False
    
    # determine the correct data type
    try:
        new_value = ast.literal_eval(new_value.strip())
    except (ValueError, SyntaxError):
        raise ValueError(print(f"Invalid value: {new_value}"))
    
    # convert line/column information into text positions
    lines = source.splitlines(keepends=True)
    line_starts = []
    position = 0
    for line in lines:
        line_starts.append(position)
        position += len(line)
    
    start_line = int(location_row["Start Line"])
    end_line = int(location_row["End Line"])
    start_column = int(location_row["Start Column"])
    end_column = int(location_row["End Column"])
    start_position = (line_starts[start_line-1]+start_column)
    end_position = (line_starts[end_line-1]+end_column)
    
    # # find the value in the file that we need to replace
    # node_value = None
    
    # verify source text matches
    curr_source_text = source[start_position:end_position] # sees what text is inside this range
    stored_source_text = location_row["Source Text"]

    if (curr_source_text.strip() != stored_source_text.strip()):
        raise ValueError ("Source file has changed. Must scan again in order to input New Value")
    
    # update the source code based on the predetermined data type found earlier
    updated_source = (source[:start_position] + repr(new_value) + source[end_position:])
    ast.parse(updated_source, filename=str(path))

    # safety feature: backup file
    backup_path = Path(str(path)+".old_version")
    shutil.copy2(path, backup_path)
    # save the updated file
    path.write_text(updated_source, encoding="utf-8")


def apply_changes(configurations_file, parameters_file, locations_file):
    """ Apply the changes in parameters.csv """
    # load the csv files
    configurations, _ = read_csv(configurations_file)
    parameters, parameter_columns = read_csv(parameters_file) 
    locations, _ = read_csv(locations_file)

    # create dictionaries to quickly identify rows
    configuration_lookup = {row["Config ID"]: row for row in configurations}
    location_lookup = {(row["Config ID"], row["Parameter"]): row for row in locations}

    # write to parameters.csv to change the Current Value and Status
    for parameter_row in parameters:
        new_value = parameter_row["New Value"]
        if new_value == "":
            continue
        can_edit = parameter_row["Can Edit"]
        if can_edit != "True":
            parameter_row["Status"] = "Cannot edit"
            continue
        
        key = (parameter_row["Config ID"], parameter_row["Parameter"])
        configuration = configuration_lookup.get(parameter_row["Config ID"])
        location = location_lookup.get(key)
        if configuration == None:
            parameter_row["Status"] = "Error configuration not found"
            continue
        if location == None:
            parameter_row["Status"] = "Error location not found"
            continue
        
        # modify the file with the new value
        try:
            update_config_parameter(file_name=configuration["File"], location_row=location, new_value=new_value)
            parameter_row["Current Value"] = new_value
            parameter_row["New Value"] = ""
            parameter_row["Status"] = "Updated"
            print(f"Updated {key[0], key[1]}")
        except ValueError as error:
            parameter_row["Status"] = f"Error: {error}"
            print(error)
        
    write_csv(parameters, parameter_columns, "parameters.csv")
