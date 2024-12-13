#!/usr/bin/env python3

import rospy
import os 
def run_rosbag_files():
    """Function to run the rosbag files one-one from the list of rosbag files.
    """
    current_dir = os.path.dirname(os.path.realpath(__file__))
    rosbag_file_path = current_dir + "/../config/rosbag_files_pc.txt"   
    file = open(rosbag_file_path, "r")
    for line in file:
        line = line.strip()
        if line == "":
            continue
        print("Running rosbag file: ", line)
        os.system("rosbag play " + line)
        print("Done running rosbag file: ", line)
        
    file.close()

if __name__ == '__main__':
    rospy.init_node('run_rosbag_files')
    run_rosbag_files()
    rospy.spin()
