import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid 
from visualization_msgs.msg import Marker

class AnalyzeMap():
    def __init__(self):
        # var
        self.rviz_pos_x = None
        self.rviz_pos_y = None
        self.resolution = None
        self.width = None
        self.height = None
        self.data = None
        self.origin_x = None
        self.origin_y = None

        # init
        rospy.init_node("analyse_map", anonymous=True)

        # sub/pub
        self.sphere_marker_pub = rospy.Publisher("/index_cell",Marker)
        self.map_sub = rospy.Subscriber("/map",OccupancyGrid,self.map_callback)

    def map_callback(self,msg):
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.data = msg.data
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        rospy.loginfo("map: resolution: {0} width:{1} height: {2} data_size: {3}".format(self.resolution,self.width,self.height,len(self.data)))

        self.mark_the_first_occupied_cell()
        self.mark_all_occupied_cells()


    def mark_the_first_occupied_cell(self):
        #
        map_data_to_set = set(self.data)
        rospy.loginfo("map values: {0}".format(map_data_to_set))
        
        index_first_occupied_cell = self.data.index(100)
        #index_first_occupied_cell = 0
        rospy.loginfo("Index of first occupied cell: {0}".format(index_first_occupied_cell))

        # The position of the first occupied cell in the map data matrix
        mat_pos_x = index_first_occupied_cell%self.width
        mat_pos_y = int(index_first_occupied_cell/self.width) # matrix position couldn't be float value
        rospy.loginfo("Position of first occupied cell in the map matrix: ({0},{1})".format(mat_pos_x,mat_pos_y))

        # where is index_of_first_matrix relative to origin/m
        rviz_pos_x = self.origin_x + mat_pos_x * self.resolution
        rviz_pos_y = self.origin_x + mat_pos_y * self.resolution
        rospy.loginfo("Position of first occupied cell in rviz: ({0},{1})".format(rviz_pos_x,rviz_pos_y))

        # 
        self.rviz_pos_x = rviz_pos_x
        self.rviz_pos_y = rviz_pos_y

    def mark_all_occupied_cells(self):
        #
        map_to_np = np.array(self.data)
        filter_occupied_cells = np.where(map_to_np == 100)[0]
        rospy.loginfo("len of list of all occupied cells: {0})".format(len(filter_occupied_cells)))

    def down_sample_map(self):
        # get a ration and down sample map
        pass

    def cut_map_with_obstacle(self):
        # get map with only the obstacles in it
        pass

    def insert_obstacle(self):
        
        pass

    def delete_obstacle(self):
        # remove all obstacles from a given rectangle
        pass

    def inflate_rectangle(self):
        # inflate all obstacles in the given rectangle
        pass
        
    def sphere_marker(self):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 0.6
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.rviz_pos_x
        marker.pose.position.y = self.rviz_pos_y
        marker.pose.position.z = 0.1

        self.sphere_marker_pub.publish(marker)
        

    def run_it(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.sphere_marker()
            rate.sleep()

if __name__ == "__main__":
    app = AnalyzeMap()
    app.run_it()
