__all__ = ["generate_ctrl_files"]

import re
import os
import json
import shutil
import subprocess
from glob import glob


def generate_ctrl_files():

    ctrl_dir_path = _get_ctrl_directory_path()
    if ctrl_dir_path is not None:
        _create_directories(ctrl_dir_path)
        # _create_ros_files(ctrl_dir_path)
        # _create_python_files(ctrl_dir_path)


# Input checker methods

def _get_ctrl_directory_path():

    ctrl_dir_path = None
    while ctrl_dir_path is None:
        print("Type 'custom' to manually enter a path to create the directory and files or\n"
              "type 'set' to create the directory and files in the set_ups directory or\n"
              "type 'back' to not continue with the program.")
        option = input("Enter one of the options above: ")
        if option == 'custom':
            ctrl_dir_path = _ctrl_path_from_custom()
        elif option == 'set':
            ctrl_dir_path = _ctrl_path_from_cwd()
        elif option == 'back':
            break
        else:
            print("'{}' is not a valid option".format(option))
    return ctrl_dir_path


def _ctrl_path_from_cwd():
    """
    Creates the controller directory path by using the current working
    directory (cwd).
    """

    ctrl_dir = _get_directory_name()
    if ctrl_dir is not None:
        return os.path.join(os.getcwd(), "set_ups", ctrl_dir)  # Return path to the input directory
    return


def _ctrl_path_from_custom():

    input_path = input("Enter a path to create the control directory: ")
    if os.path.exists(input_path):
        ctrl_dir = _get_directory_name()
        if ctrl_dir is not None:
            return os.path.join(input_path, ctrl_dir)
    print("The path you entered does not exist. Please enter a valid path.\n")
    return


def _get_directory_name():

    dir_name_regex = "[_a-zA-Z][_a-zA-Z0-9]*"

    ctrl_dir = input("Enter the name of the control directory: ")
    if re.fullmatch(dir_name_regex, ctrl_dir):
        return ctrl_dir  # Return input directory name
    print("Please enter a valid directory name. An example of a"
          " valid directory name is 'movement_control'.\n")
    return


# Directory generations methods

def _create_directories(ctrl_dir_path):

    os.mkdir(ctrl_dir_path)  # Create the control directory

    # Create directories to store ROS message and service files
    for item_dir in ["msg", "srv"]:
        dir_path = os.path.join(ctrl_dir_path, item_dir)
        os.mkdir(dir_path)


# File creation methods

def _create_ros_files(ctrl_dir_path):
    # Add subprocess stuff here to run and (maybe) create the packages
    pass


def _create_python_files(ctrl_dir_path):

    # Get code template from text file
    code_temp_path = os.path.join(os.getcwd(), "utils", "ctrl_file_generator", "code_temps.txt")
    with open(code_temp_path) as code_temps:
        templates = json.loads(code_temps.read())

    # Create new files and pass the templates
    for code_name in ["comms_setup", "ctrl_setup", "runner_gen"]:
        code_path = os.path.join(ctrl_dir_path, code_name + ".py")  # Get paths to relevant file paths
        with open(code_path, "w") as code_file:
            code_file.write(templates[code_name])
