

from __future__ import print_function


__all__ = ["generate_ctrl_files"]


import re
import os
import sys
import json
import subprocess

# TODO: Make re.match behave like re.fullmatch by probably using anchors

# Define version specific functions
if sys.version_info[:2] <= (2, 7):  # If Python 2.7 or lower
    match = re.match
    user_input = raw_input
else:
    user_input = input
    match = re.fullmatch


def generate_ctrl_files():

    ctrl_dir_path = _get_ctrl_directory_path()
    if ctrl_dir_path is not None:
        _create_directories(ctrl_dir_path)
        _create_ros_files(ctrl_dir_path)
        _create_python_files(ctrl_dir_path)


# Control directory path creator functions

def _get_ctrl_directory_path():

    ctrl_dir_path = None
    while ctrl_dir_path is None:
        print("Type 'set' to create the directory and files in the 'src' directory or\n"
              "type 'back' to not continue with the program.")
        option = user_input("Enter one of the options above: ")
        if option == 'set':
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
        return os.path.join(os.getcwd(), "src", ctrl_dir)  # Return path to the input directory
    return


def _get_directory_name():

    dir_name_regex = "[_a-zA-Z][_a-zA-Z0-9]*"

    ctrl_dir = user_input("Enter the name of the control directory: ")
    if match(dir_name_regex, ctrl_dir):
        return ctrl_dir  # Return input directory name
    print("Please enter a valid directory name. An example of a"
          " valid directory name is 'movement_control'.\n")
    return


# Directory generations functions

def _create_directories(ctrl_dir_path):

    print("Creating directories...")

    os.mkdir(ctrl_dir_path)  # Create the control directory

    # Create directories to store ROS message and service files
    for item_dir in ["msg", "srv"]:
        dir_path = os.path.join(ctrl_dir_path, item_dir)
        os.mkdir(dir_path)


# Create ROS related files functions

_DEFAULT_DEPEND = ('std_msgs', 'rospy', 'roscpp')  # Default dependencies that need to be added


def _create_ros_files(ctrl_dir_path):
    # Add subprocess stuff here to run and (maybe) create the packages

    print("This section is to add/remove any ROS dependencies.\n")

    add_depend = []
    while True:
        print("Type 'add' to add a new ROS dependency\n"
              "type 'remove' to remove an added dependency\n"
              "type 'end' to stop adding or removing dependencies")
        option = user_input("Type one of the options above here:")
        if option == "add":
            _add_ros_dependency(add_depend)
        elif option == "end":
            break
        elif option == "remove":
            _remove_ros_dependency(add_depend)

    _run_commands(add_depend, ctrl_dir_path)


def _add_ros_dependency(add_depend):

    _display_dependencies(add_depend)
    pkg = user_input("Add a ROS package dependency:")
    if any(pkg in depend_seq for depend_seq in [add_depend, _DEFAULT_DEPEND]):
        print("The package is already present in the dependencies.")
    else:
        add_depend.append(pkg)


def _remove_ros_dependency(add_depend):

    _display_dependencies(add_depend)
    pkg = user_input("Remove a ROS package dependency:")
    if pkg in add_depend:
        add_depend.remove(pkg)
    else:
        print("The package {} is not present in the modifiable dependencies.".format(pkg))


def _run_commands(add_depend, ctrl_dir_path):

    print("Creating ROS files and package...")

    add_depend.extend(_DEFAULT_DEPEND)
    cmd_1 = 'cd src'  # Move to src directory in controller_pkg
    cmd_2 = "catkin_create_pkg " + os.path.basename(ctrl_dir_path) + " " + " ".join(add_depend)  # Create ROS package
    ros_pkg_p = subprocess.Popen(cmd_1 + "; " + cmd_2, shell=True)
    ros_pkg_p.wait()

    os.rmdir(os.path.join(ctrl_dir_path, 'src'))  # Remove src directory that comes with ROS package


def _display_dependencies(add_depend):

    print("All the dependencies are:", ', '.join(add_depend + list(_DEFAULT_DEPEND)))
    if len(add_depend) == 0:
        print("There are no dependencies you can modify at the moment.")
    else:
        print("The modifiable dependencies are:", ', '.join(add_depend))


# Function that creates the python files

def _create_python_files(ctrl_dir_path):

    print("Creating python files...")

    # Get code template from text file
    code_temp_path = os.path.join(os.getcwd(), "utils", "ctrl_file_generator", "code_temps.txt")
    with open(code_temp_path) as code_temps:
        templates = json.loads(code_temps.read(), strict=False)

    # Create new files and pass the templates
    for code_name in ["comms_setup", "ctrl_setup", "runner_gen"]:
        code_path = os.path.join(ctrl_dir_path, code_name + ".py")  # Get paths to relevant file paths
        with open(code_path, "w") as code_file:
            code_file.write(templates[code_name])
