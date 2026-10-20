import ast
from pathlib import Path # modern system for managing file paths


def extract_one_config(file_name):
    """  Extract parameters from one file under the Config directory. """
    path = Path(file_name)
    source = path.read_text(encoding="utf-8") # utf-8 translates characters into binary representation
    tree = ast.parse(source) # breaks down the binary text into a hierarchical tree of objects

    # list of dictionaries to store data of each constant
    configurations = []
    parameters = []
    parameters_locations = []
    
    for node in ast.walk(tree): # loops through each line in the tree (excludes comments)
        # filters out AST nodes that are not part of the search
        
        if not isinstance(node, ast.Assign): # supresses everything except for assignemnts
            continue
        if not isinstance(node.value, ast.Call): # supresses everything down to Call
            continue 
        function_call = node.value

        # Extract the class name
        class_name = None
        if isinstance(function_call.func, ast.Name): # extracts simple names
            class_name = function_call.func.id
        elif isinstance(function_call.func, ast.Attribute): # extracts dotted names
            class_name = function_call.func.attr
        
        # skips these functions in MainConfigs/SR18
        if class_name == "loadConfigs":
            continue

        # Find the object name 
        object_name = "-" # if dash then the object name was not found
        if isinstance(node.targets[0], ast.Name):
            object_name = node.targets[0].id
        vehicle_part = get_vehicle_part(path)
        configurations.append({"Config ID": f"{vehicle_part}-{object_name}", "Part": vehicle_part, "File": str(path), "Object": object_name, "Config Class": class_name})

        # Extract keyword arguments
        can_edit = False
        for keyword in function_call.keywords:
            node_value = keyword.value
            # Ignores **kwargs bc they don't have values that need to be changed
            if keyword.arg is None:
                continue
            can_edit = can_edit_value(node_value)

            # adds metadata to dictionaries
            parameters.append({"Config ID": f"{vehicle_part}-{object_name}", "Can Edit": can_edit, "Parameter": keyword.arg, 
                                "Current Value": ast.unparse(keyword.value), "New Value": "", "Status": "-"})
        
            parameters_locations.append({"Config ID": f"{vehicle_part}-{object_name}", "Parameter": keyword.arg, "Start Line": node_value.lineno, "End Line": node_value.end_lineno, "Start Column": node_value.col_offset, "End Column": node_value.end_col_offset, "Source Text": ast.get_source_segment(source, node_value)})

    # Extract default values from configuration classes
    for class_node in ast.walk(tree):
        if not isinstance(class_node, ast.ClassDef): # supresses down to only ClassDef node
            continue
        # Search every function in the class
        for function_node in ast.walk(class_node):
            if not isinstance(function_node, ast.FunctionDef):
                continue
            
            vehicle_part = get_vehicle_part(path)
            config_id = (f"{vehicle_part}-"f"{class_node.name}.{function_node.name}")
            configurations.append({"Config ID": config_id, "Part": vehicle_part, "File": str(path), "Object": f"{class_node.name}.{function_node.name}", "Config Class": class_node.name})
            
            # finds where the first index is for default parameters
            first_default_index = (len(function_node.args.args) - len(function_node.args.defaults))
            # pairs each argument with its value (eg. initial_soc=None)
            for argument, node_value in zip(function_node.args.args[first_default_index:], function_node.args.defaults):
                can_edit = can_edit_value(node_value)
                parameters.append({"Config ID": config_id, "Can Edit": can_edit, "Parameter": argument.arg, 
                                "Current Value": ast.unparse(node_value), "New Value": "", "Status": "-"})
                parameters_locations.append({"Config ID": config_id, "Parameter": argument.arg, "Start Line": node_value.lineno, "End Line": node_value.end_lineno, "Start Column": node_value.col_offset, "End Column": node_value.end_col_offset, "Source Text": ast.get_source_segment(source, node_value)})


    return configurations, parameters, parameters_locations



def can_edit_value(node_value):
    """ Check if an AST value can be edited """
    try:
        value_text = ast.unparse(node_value)
        ast.literal_eval(value_text)
        return True
    except (ValueError, SyntaxError):
        return False

def extract_all_configs(vehicle_dir):
    """ Extract parameters from all files under the Vehicle directory. """
    vehicle_root = Path(vehicle_dir) 
    all_configurations = []
    all_parameters = []
    all_parameters_locations = []
    
    # find all subdirectories in Vehicle
    for py_file in vehicle_root.rglob("*.py"):
        configurations, parameters, parameters_locations = extract_one_config(py_file)
        # skip the build files
        if "__pycache__" in py_file.parts:
            continue

        all_configurations.extend(configurations)
        all_parameters.extend(parameters)
        all_parameters_locations.extend(parameters_locations)
    
    # sorts the rows based on the Can Edit column
    # True values come before False values
    all_parameters.sort(key=lambda row: row["Can Edit"] == False)

    return all_configurations, all_parameters, all_parameters_locations



def get_vehicle_part(path):
    if "Configs" in path.parts:
        # Path: Vehicle/*/Configs/...
        configs_index = path.parts.index("Configs") 
        return path.parts[configs_index-1] # returns * (vehicle part)
    if "MainConfigs" in path.parts:
        # Path: MainConfigs/SR18.py
        return "MainConfigs"
    # Path: Configs/*/... 
    return path.parent.name # return * (vehicle part)
 


