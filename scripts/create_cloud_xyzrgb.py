#!/usr/bin/env python3
# PointCloud2 color cube
# https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
import rospy
import struct
import numpy as np
import random

from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def talker():
    pub = rospy.Publisher("cloud_in", PointCloud2, queue_size=2)

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              # PointField('rgb', 12, PointField.UINT32, 1),
              PointField('rgba', 12, PointField.UINT32, 1),
              ]


    header = Header()
    header.frame_id = "map"


    lim = 3
    while not rospy.is_shutdown():
        points = []
        for i in range(lim):
            for j in range(lim):
                for k in range(lim):
                    x = float(i) / lim
                    y = float(j) / lim
                    z = float(k) / lim
                    r = int(x * 255.0)
                    g = int(y * 255.0)
                    b = int(z * 255.0)
                    a = 255
                    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
                    x=x*np.random.random_sample()*1.5
                    z=z * np.random.random_sample()*2.0
                    y = y * random.random()*2.0
                    pt = [x, y, z, rgb]
                    points.append(pt)


        pc2 = point_cloud2.create_cloud(header, fields, points)


        pc2.header.stamp = rospy.Time.now()
        pub.publish(pc2)
        rospy.sleep(1.0)


if __name__ == '__main__':
    rospy.init_node("create_cloud_xyzrgb")
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
