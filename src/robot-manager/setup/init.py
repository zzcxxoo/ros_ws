"""this file is for CICD first-time installation 
The sample file only contains copy sample config file to jtcx config folder.
Feel free to make modifications if requried. 
This file should be run after catkin_make and source.
"""
import os
import sys
from re import sub
import rospkg
import getpass
import subprocess

# ! conf_folder MUST follow jtcx standard
conf_folder = "/usr/local/params"
# ! pkg_name MUST be ros_pkg name
pkg_name = "robot_manager"
user_name = getpass.getuser()

# ! STEP 1: creat config dir if not exist
#  config_dir MUST follow jtcx standard: $conf_folder/$pkg_name
conf_dir = os.path.join(conf_folder, pkg_name)
a = subprocess.Popen(['sudo mkdir -p ' + conf_dir], shell=True)
a.wait()

# ! STEP 2: copy sample config file to the dir created in step 2
# all sample config files MUST put into config folder of your repo
rospack = rospkg.RosPack()
sample_config_dir = os.path.join(rospack.get_path(pkg_name), 'config')
a = subprocess.Popen(['sudo cp -ar ' + sample_config_dir + '/* ' + conf_dir], shell=True)
a.wait()

a = subprocess.Popen([f'sudo rm -rf {conf_dir}/robot'], shell=True)
a.wait()
robot_dir = os.path.join('/home', user_name, '.robot')
sample_robot_dir = os.path.join(rospack.get_path(pkg_name), 'config/robot')
a = subprocess.Popen(['sudo cp -ar ' + sample_robot_dir + ' ' + robot_dir], shell=True)
a.wait()
a = subprocess.Popen([f'sudo chown -R {user_name}:{user_name} {robot_dir}'], shell=True)
a.wait()

# ! STEP 3: copy health_monitor to designated folder "/usr/local/share"
init_folder = rospack.get_path(pkg_name)
health_monitor_dir = os.path.join(init_folder, "health_monitor")
a = subprocess.Popen([f'sudo cp -ar {health_monitor_dir} /usr/local/share'], shell=True)
a.wait()

# ! STEP 4: copy systemd service files to designated folder "/etc/systemd/system/"
systemd_folder = os.path.join(init_folder, "systemd")
a = subprocess.Popen([f'sudo cp -a {systemd_folder}/*.service /etc/systemd/system'], shell=True)
a.wait()

# ! make log dir
log_dir = "/var/log/jtcx/robot-manager/"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)

# ! STEP 5: enable service for auto start when power on
a = subprocess.Popen(['sudo systemctl daemon-reload'], shell=True)
a.wait()

a = subprocess.Popen(['sudo systemctl enable roscore.service'], shell=True)
a.wait()

a = subprocess.Popen(['sudo systemctl enable control_master.service'], shell=True)
a.wait()

# ! STEP 6: start jtcx programs
a = subprocess.Popen(['sudo systemctl start roscore'], shell=True)
a.wait()