#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from map_ugly_tools.msg import DeleteRectObstacle

class DeleteObstacleWithinRect():
    def __init__(self):
        # var
        self.resolution = None
        self.width = None
        self.height = None
        self.data = None
        self.origin = None
   

        self.new_map = OccupancyGrid()

        # init
        rospy.init_node("delete_obstacle_with_rect", anonymous=True)

        # sub/pub
        self.map_rect_pub = rospy.Publisher("/map_rect", OccupancyGrid,queue_size=10)
        self.map_sub = rospy.Subscriber("/map",OccupancyGrid,self.map_callback)
        self.rect_sub = rospy.Subscriber("/rect",DeleteRectObstacle, self.delete_rect_obstacle_callback)
        
    def map_callback(self,msg):
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin = msg.info.origin
        self.data = msg.data
        self.new_map = msg

        rospy.loginfo("Map info: resolution: {0}meter/pixel width:{1}pixels height: {2}pixels data_size: {3}pixels".format(self.resolution,self.width,self.height,len(self.data)))


    def delete_rect_obstacle_callback(self, msg):
        #..
        rospy.loginfo("new rect data arrived!")
        point_1_x = msg.point_1.x
        point_1_y = msg.point_1.y
        point_2_x = msg.point_2.x
        point_2_y = msg.point_2.y

        #..get distance in meter from the bottom left corner of the map
        point_1_x_d = -1 * ( self.origin.position.x - point_1_x)
        point_1_y_d = -1 * ( self.origin.position.y - point_1_y)
        point_2_x_d = -1 * ( self.origin.position.x - point_2_x)
        point_2_y_d = -1 * ( self.origin.position.y - point_2_y)

        #.. get corresponding pixel values
        pixel_1 = int((point_1_x_d / self.resolution) * (point_1_y_d / self.resolution))
        pixel_2 = int((point_2_x_d / self.resolution) * (point_2_y_d / self.resolution))

        pixel_small = pixel_1 if pixel_1 < pixel_2 else pixel_2
        pixel_large = pixel_1 if pixel_1 > pixel_2 else pixel_2

        #.. 
        h_gap = int(pixel_small / self.width)
        w_1 = pixel_small % self.width
        w_2 = pixel_large % self.width
        w_gap = w_1 if w_1 < w_2 else w_2

        h_rect = int(pixel_large/ self.width) - h_gap
        w_rect = (w_1 if w_1 > w_2 else w_2) - w_gap

        rospy.loginfo("h_gap:{0} w_gap:{1} h_rect: {2} w_rect {3}".format(h_gap, w_gap, h_rect, w_rect))


        #..
        map_data_np = np.array(self.data)
        map_original_np = np.array(self.data)
        for j in range(h_rect):
            rospy.loginfo("Befor len of occupied cell: {0} j: {1}".format(len(np.where(map_data_np[(h_gap + j) * self.width + w_gap: (h_gap + j) * self.width + w_gap + w_rect] == 100)), j))
            map_data_np[(h_gap + j) * self.width + w_gap: (h_gap + j) * self.width + w_gap + w_rect] = 100
            rospy.loginfo("After len of occupied cell: {0} j: {1}".format(len(np.where(map_data_np[(h_gap + j) * self.width + w_gap: (h_gap + j) * self.width + w_gap + w_rect] == 100)), j))

        self.new_map.header.frame_id = "map"
        self.new_map.info.width = self.width
        self.new_map.info.height = self.height
        self.new_map.info.origin = self.origin
        self.new_map.data = map_data_np

        rospy.loginfo("Original map number of obstacle pixels: {0}".format(len(np.where(map_original_np == 100)[0])))
        rospy.loginfo("rect map number of obstacle pixels: {0}".format(len(np.where(map_data_np == 100)[0])))

         
    def run_it(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.map_rect_pub.publish(self.new_map)
            rate.sleep()

if __name__ == "__main__":
    app = DeleteObstacleWithinRect()
    app.run_it()
