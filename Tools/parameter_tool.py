from scanner import extract_all_configs
from csv_manager import write_csv
from updater import apply_changes
import argparse

def scan(config_directory):
    """ Scan all files under the Configs directory"""
    configurations, parameters, parameters_locations = extract_all_configs(config_directory)

    config_cols = ["Config ID", "Part", "File", "Object", "Config Class"]
    param_cols = ["Config ID", "Can Edit", "Parameter", "Current Value", "New Value", "Status"]
    param_loc_cols = ["Config ID", "Parameter", "Start Line", "End Line", "Start Column", "End Column", "Source Text"]

    write_csv(configurations, config_cols, "configurations.csv")
    write_csv(parameters, param_cols, "parameters.csv")
    write_csv(parameters_locations, param_loc_cols, "parameters_locations.csv")



def main():
    parent_parser = argparse.ArgumentParser()

    sub_commands = parent_parser.add_subparsers(dest="sub_command_name", required=True)

    scan_command = sub_commands.add_parser("scan")
    # no need to specify Config's root directory unless it's not Vehicle
    scan_command.add_argument("--directory", default="Vehicle", help="Directory containing vehicle configuration files") 

    apply_command = sub_commands.add_parser("apply")

    args = parent_parser.parse_args()

    if args.sub_command_name == "scan":
        scan(args.directory)
    elif args.sub_command_name == "apply":
        apply_changes("configurations.csv", "parameters.csv", "parameters_locations.csv")
    
if __name__=="__main__":
    main()


