#!/usr/bin/env python3

import rospy
import rosbag
import sys

if sys.getdefaultencoding() != 'utf-8':
    reload(sys)
    sys.setdefaultencoding('utf-8')
bag_name = '2021-04-22-11-15-44.bag'
out_bag_name = 'out_2021-04-22-11-15-44.bag'
dst_dir = '/home/moriarty/Files/Datasets/lidarfusion/2021-04-22-beijing-86-fusion/'

with rosbag.Bag(dst_dir+out_bag_name, 'w') as outbag:
    stamp = None
    for topic, msg, t in rosbag.Bag(dst_dir+bag_name).read_messages():
        if (topic != '/tf'):
            outbag.write(topic, msg, msg.header.stamp)

print("finished")

