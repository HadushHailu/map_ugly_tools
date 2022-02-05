#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid 
from visualization_msgs.msg import Marker

class DownSampleMapEtc():
    def __init__(self):
        # var
        self.original_header = None
        self.original_resolution = None
        self.original_width = None
        self.original_height = None
        self.original_origin = None
        self.original_data = []

        self.new_resolution = None
        self.new_height = None
        self.new_width = None
        self.new_origin = None
        self.new_data = []

        self.step_size = 6

        self.new_map = OccupancyGrid()

        # init
        rospy.init_node("down_sample_map_etc", anonymous=True)

        # sub/pub
        self.down_sample_pub = rospy.Publisher("/down_sampled_map",OccupancyGrid,queue_size=10)
        self.map_sub = rospy.Subscriber("/map",OccupancyGrid,self.map_callback)

    def map_callback(self,msg):
        self.original_header = msg.header
        self.original_resolution = msg.info.resolution
        self.original_width = msg.info.width
        self.original_height = msg.info.height
        self.original_origin = msg.info.origin
        self.original_data = msg.data

        rospy.loginfo("Map info: resolution: {0}meter/pixel width:{1}pixels height: {2}pixels data_size: {3}pixels".format(self.original_resolution,self.original_width,self.original_height,len(self.original_data)))

        self.down_sample_map(self.step_size)


    def down_sample_map(self,step_size):
        # get a ration and down sample map
        self.new_resolution = self.original_resolution * step_size
        self.new_width = int(self.original_width / step_size)
        self.new_height = int(self.original_height / step_size)
        self.new_origin = self.original_origin

        for j in range(self.original_height):
            if j % step_size == 0:
                extract_row = self.original_data[j * self.original_width: j*self.original_width+self.original_width:step_size]
                self.new_data.append(extract_row)
        
        # self.new_data has format of [[],[],[]] so use np to flatten it
        new_data_np = np.array(self.new_data)
        self.new_data = new_data_np.flatten()

        rospy.loginfo("Down dampled map info: resolution: {0} width: {1} height: {2} data_size {3}".format(self.new_resolution, self.new_width, self.new_height,len(self.new_data)))

        self.new_map.header = self.original_header
        self.new_map.info.resolution = self.new_resolution
        self.new_map.info.width = self.new_width 
        self.new_map.info.height = self.new_height
        self.new_map.info.origin = self.new_origin
        self.new_map.data = self.new_data

    def run_it(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.down_sample_pub.publish(self.new_map)
            rate.sleep()

if __name__ == "__main__":
    app = DownSampleMapEtc()
    app.run_it()
