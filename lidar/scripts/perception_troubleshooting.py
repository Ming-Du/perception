#!/usr/bin/env python

import sys
import time
import socket
import subprocess

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from autopilot_msgs.msg import BinaryData

jv_topic_list = [
    '/perception/lidar/lidar_obstacle',
    '/sensor/lidar/middle/point_cloud',
    '/sensor/lidar/rear/point_cloud',
    '/sensor/lidar/front_right/point_cloud',
    '/sensor/lidar/front_left/point_cloud',
    '/sensor/lidar/innolidar/point_cloud',
    '/localization/global'
]
jv_driver_name_list = [
    {'topic': '/sensor/lidar/rear/point_cloud', 'driver': '/sensor/lidar/c32/rear/c32_rear_decoder', 'ip': '192.168.1.203'},
    {'topic': '/sensor/lidar/front_right/point_cloud', 'driver': '/sensor/lidar/c32/front_right/c32_right_decoder', 'ip': '192.168.1.202'},
    {'topic': '/sensor/lidar/front_left/point_cloud', 'driver': '/sensor/lidar/c32/front_left/c32_left_decoder', 'ip': '192.168.1.201'},
    {'topic': '/sensor/lidar/innolidar/point_cloud', 'driver': '/sensor/lidar/innolidar/drivers_innolidar', 'ip': '192.168.1.205'},
]
m2_topic_list = [
    '/perception/lidar/lidar_zvision_obstacle',
    '/sensor/zvisionlidar/middle/point_cloud',
    '/sensor/lidar/zvisionlidar/front/point_cloud',
    '/sensor/lidar/zvisionlidar/left/point_cloud',
    '/sensor/lidar/zvisionlidar/rear/point_cloud',
    '/sensor/lidar/zvisionlidar/right/point_cloud',
    '/sensor/lidar/innolidar/point_cloud',
    '/localization/global'
]
m2_driver_name_list = [
    {'topic': '/sensor/lidar/zvisionlidar/front/point_cloud', 'driver': '/zvision_lidar_front_nodelet_manager', 'ip': '192.168.1.200'},
    {'topic': '/sensor/lidar/zvisionlidar/left/point_cloud', 'driver': '/zvision_lidar_left_nodelet_manager', 'ip': '192.168.1.201'},
    {'topic': '/sensor/lidar/zvisionlidar/rear/point_cloud', 'driver': '/zvision_lidar_rear_nodelet_manager', 'ip': '192.168.1.202'},
    {'topic': '/sensor/lidar/zvisionlidar/right/point_cloud', 'driver': '/zvision_lidar_right_nodelet_manager', 'ip': '192.168.1.203'},
    {'topic': '/sensor/lidar/innolidar/point_cloud', 'driver': '/sensor/lidar/innolidar/drivers_innolidar', 'ip': '192.168.1.205'}
]
topic_list = []
driver_name_list = []

def check_node_existence(node_name):
    try:
        cmd = ['rosnode', 'ping', node_name]
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        time.sleep(1)
        process.terminate()  
        output = process.communicate()[0] 
        lines = output.split('\n')
        for line in lines:
            if "ERROR" in line or "unknown node" in line:
                return False
        return True
    except Exception as e:
        return False
    
class TopicFrequencyChecker:
    def __init__(self):
        rospy.init_node('topic_frequency_checker', anonymous=True)
        self.topic_list = topic_list
        self.message_count = {topic: 0 for topic in self.topic_list}
        self.frequency = {topic: None for topic in self.topic_list}
        self.last_time = rospy.Time.now()
        self.lidar_timestamp = None
        self.global_timestamp = None
        self.time_diff = None

        rospy.Subscriber(self.topic_list[0], String, lambda msg, t=self.topic_list[0]: self.callback(msg, t))
        for topic in self.topic_list[1:-1]:
            rospy.Subscriber(topic, PointCloud2, lambda msg, t=topic: self.callback(msg, t))
        
        rospy.Subscriber(self.topic_list[-1], BinaryData, self.global_callback)
        
        self.shutdown_timer = rospy.Timer(rospy.Duration(2), self.shutdown_callback)
        rospy.spin()

    def callback(self, data, topic):
        current_time = rospy.Time.now()
        elapsed_time = current_time - self.last_time
        self.message_count[topic] += 1

        if (self.frequency[topic] is None or elapsed_time.to_sec() > 1.0) and self.message_count[topic] >= 5:
            self.frequency[topic] = self.message_count[topic] / elapsed_time.to_sec()

    def global_callback(self, data):
        self.global_timestamp = data.header.stamp

        if self.lidar_timestamp is not None:
            self.time_diff = abs((self.global_timestamp - self.lidar_timestamp).to_sec())

    def shutdown_callback(self, event):
        rospy.signal_shutdown("")

    def get_time_diff(self):
        return self.time_diff

def query_ip_address():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.connect(('114.114.114.114', 80))
        ip_address = sock.getsockname()[0]
    finally:
        sock.close()
    
    return ip_address

def ping_target(target_ip):
    try:
        cmd = ["ping", "-c", "1", "-W", "1", target_ip]
        process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, universal_newlines=True)
        time.sleep(1)
        process.terminate()  
        output = process.communicate()[0] 
        lines = output.split('\n')
        for line in lines:
            if "1 packets transmitted, 1 received" in line:
                return True
        return None
    except Exception as e:
        return None

def check_perception_status(type = 'jinlv'):
    if type == 'jinlv':
        node_names = [
            "/perception/lidar/rs_perception_node",
            "/perception/lidar/rs_perception_falcon_node"
        ]
        perception_node_position = [
            " [ 106 ]",
            " [ 104 ]"
        ]
        lidar_node_position = " [ 106 ]"
    else:
        node_names = [
            "/perception/lidar/rs_perception_zvision_node",
            "/perception/lidar/rs_perception_falcon_node"
        ]
        perception_node_position = [
            " [ 107 ]",
            " [ 104 ]"
        ]
        lidar_node_position = " [ 107 ]"
        
    index = 0
    for node_name in node_names:
        node_exists = check_node_existence(node_name)
        if not node_exists:
            print("\033[31mNode '{}' does not exist.\033[0m\n\033[31m   The node may have crashed, please contact the developers or reboot.\033[0m".format(node_name + perception_node_position[index]))
            exit(0)
        print("Node '{}' exists.".format(node_name + perception_node_position[index]))
        index += 1

    topic_check = TopicFrequencyChecker()
    if type == 'jinlv':
        obstacle_topic_frequency = topic_check.frequency['/perception/lidar/lidar_obstacle']
        middle_topic_frequency = topic_check.frequency['/sensor/lidar/middle/point_cloud']
    else:
        obstacle_topic_frequency = topic_check.frequency['/perception/lidar/lidar_zvision_obstacle']
        middle_topic_frequency = topic_check.frequency['/sensor/zvisionlidar/middle/point_cloud']
    print("Obstacle topic frequency: {} Hz / 10Hz\nMiddle topic frequency: {} Hz / 10Hz".format(obstacle_topic_frequency, middle_topic_frequency))
    if obstacle_topic_frequency and obstacle_topic_frequency <= 9 and middle_topic_frequency and middle_topic_frequency > 9:
        print("\n\033[31mUnknown reason, please contact the developers or reboot.\033[0m")
        exit(0)
    elif not obstacle_topic_frequency and middle_topic_frequency and middle_topic_frequency > 9:
        time_diff = topic_check.get_time_diff()
        if time_diff and time_diff > 0.3:
            print("\n\033[31mLocation and middle timestamp difference more than 0.3s, please contact the developers.\033[0m")
        else:
            print("\n\033[31mnknown reason, please contact the developers or reboot.\033[0m")
        exit(0)
    elif (not obstacle_topic_frequency or obstacle_topic_frequency <= 9) and (not middle_topic_frequency or middle_topic_frequency <= 9):
        if type == 'jinlv':
            topic_name = "/xiaoba_lidars_fusion"
            position = " [ 106 ]"
        else:
            topic_name = "/xiaoba_zvisionlidars_fusion"
            position = " [ 107 ]"
        node_exists = check_node_existence(topic_name)
        if node_exists and middle_topic_frequency:
            print("Node '{}' exists.\n\033[31mTime unsynchronizated, please contact the developers.\033[0m".format(topic_name + position))
            exit(0)
        elif not node_exists:
            print("\033[31mNode '{}' does not exist.\033[0m\n\033[31m   The node may have crashed, please contact the developers or reboot.\033[0m".format(topic_name + position))
            exit(0)
            
        for driver_name in driver_name_list:
            topic_frequency = topic_check.frequency[driver_name['topic']]
            print("Lidar topic: '{}', frequency: {} Hz / 10Hz".format(driver_name['topic'], topic_frequency))
            if not topic_frequency:
                node_exists = check_node_existence(driver_name['driver'])
                if not node_exists:
                    print("\033[31mNode '{}' does not exist.\033[0m\n\033[31m   The node may have crashed, please contact the developers or reboot.\033[0m".format(driver_name['driver'] + lidar_node_position))
                else:
                    print("Node '{}' exists.".format(driver_name['driver'] + lidar_node_position))
                    ret = ping_target(driver_name['ip'])
                    if ret:
                        print("   Lidar topic: '{}', ip '{}' ping success.".format(driver_name['topic'] + lidar_node_position ,driver_name['ip']))
                    else:
                        print("\033[31m   Lidar topic: '{}', ip '{}' ping failed.\033[0m\n\033[31m   Lidar disconnected, please contact the developers.\033[0m".format(driver_name['topic'] + lidar_node_position,driver_name['ip']))
                exit(0)            

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("\033[31mplease input valid vehicle type:\033[0m")
        print("\033[31m     B1: b1, B1, b, B, 1\033[0m")
        print("\033[31m     B2: b2, B2, m2, M2, m, M, 2\033[0m")
        exit(0)
        
    ip_address = query_ip_address()
    print("Local Machine IP: '{}' ".format(ip_address))
    
    if sys.argv[1] in ['b1', 'B1', 'b', 'B', '1']:
        topic_list = jv_topic_list
        driver_name_list = jv_driver_name_list
        check_perception_status()
    elif sys.argv[1] in ['b2', 'B2', 'm2', 'M2', 'm', 'M', '2']:
        topic_list = m2_topic_list
        driver_name_list = m2_driver_name_list
        check_perception_status('m2')
