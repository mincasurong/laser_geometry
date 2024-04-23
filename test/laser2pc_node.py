#!/usr/bin/env python

import rospy
import math
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, LaserScan
import laser_geometry.laser_geometry as lg

class FunctionContainer:
    def __init__(self):
        self.idx = 0
        self.lp = lg.LaserProjection()


    def scan_cb(self, msg):
        
        pc2_msg = self.lp.projectLaser(msg) # convert the message of type LaserScan to a PointCloud2

        ''' I do not neet to publish the msg'''
        # pc_pub.publish(pc2_msg)
        
        
        point_generator = pc2.read_points(pc2_msg) # convert it to a generator of the individual points
        
        
        for point in point_generator:
            pass
        print(point) # To watch each points
        point_list = pc2.read_points_list(pc2_msg) # or a list of the individual points which is less efficient        
        print(point_list[self.idx].x, point_list[self.idx].y, point_list[self.idx].index) # we can access the point list with an index, each element is a namedtuple
        
def main():
    rospy.init_node("laserscan_to_pointcloud")
    function_container = FunctionContainer()

    rospy.Subscriber("/robot/front_laser/scan", LaserScan, function_container.scan_cb, queue_size=1)
    rospy.spin()    

if __name__ == '__main__':
    ''' I do not need to publish the topic '''
    # pc_pub = rospy.Publisher("/robot/laser/pointcloud", PointCloud2, queue_size=1)
    main()


