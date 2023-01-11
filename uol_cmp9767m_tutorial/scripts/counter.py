import rospy, image_geometry, tf
import cv2
import matplotlib
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from sklearn.cluster import DBSCAN
from sklearn.cluster import KMeans
import actionlib

from topological_navigation_msgs.msg import GotoNodeAction, GotoNodeGoal

class Counter():
    camera_model = None
    image_depth_ros = None
    visualisation = True
    color2depth_aspect = (84.1/1920) / (70.0/512)
    object_location = []

    def __init__(self):
        
        self.bridge = CvBridge()
        #Most of this code was developed from https://github.com/LCAS/CMP9767M/blob/master/uol_cmp9767m_tutorial/scripts/image_projection_3.py

        #camera topic subscribers 
        self.camera_info_sub = rospy.Subscriber('/thorvald_001/kinect2_right_camera/hd/camera_info', CameraInfo, self.camera_info_callback)
        rospy.Subscriber("/thorvald_001/kinect2_right_sensor/sd/image_depth_rect", Image, self.image_depth_callback)
        self.front_camera = rospy.Subscriber("/thorvald_001/kinect2_right_camera/hd/image_color_rect", Image, self.callback)
        self.tf_listener = tf.TransformListener()

        #Location publisher
        self.object_location_pub = rospy.Publisher('/thorvald_001/object_location', PoseStamped, queue_size=10)

    
    def camera_info_callback(self, data):
        self.camera_info_sub.unregister() #This is to subscribe just once.
        self.camera_model = image_geometry.PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        
    def image_depth_callback(self, data):
        self.image_depth_ros = data
    
    def callback(self, data):    
        if self.camera_model is None:
            return

        if self.image_depth_ros is None:
            return

        try:
            camera_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
            image_depth = self.bridge.imgmsg_to_cv2(self.image_depth_ros, desired_encoding="32FC1")
        except CvBridgeError as e:
            print(e)

        try:
            # Resize image
            test_image = cv2.resize(camera_image, None, fx=0.5, fy=0.5, interpolation = cv2.INTER_CUBIC)
            cv2.imshow("GCM", test_image)

            # Getting the region of interest (roi)
            roi = camera_image[0: 950, 1000: 1920]

            # Converting  image from RGB to HSV
            camera_image_hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            # Setting the range colour
            grape_high_range = np.array([107, 255, 255])
            grape_low_range = np.array([100, 18, 46])
            

            # Get feeds from only grape colour
            grape_colour_mask = cv2.inRange(camera_image_hsv, grape_low_range, grape_high_range)
            # Threshold HSV image to get only grape colours
            _, grape_colour_mask = cv2.threshold(grape_colour_mask, 254, 255, cv2.THRESH_BINARY)

            kernel = np.ones((5,5),np.uint8)
            dilation = cv2.dilate(grape_colour_mask, kernel, iterations = 1)

            #shape detection contour analysis
            contours, _ = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2:]


            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 250:
                    #draw a rectangle around each grape bunch
                    x,y,w,h = cv2.boundingRect(contour)
                    cv2.rectangle(roi, (x,y), (x+w, y+h), (255,255,255),2)

                    #Identifying the grapes and interpreting as map coordinate
                    M = cv2.moments(contour)

                    if M["m00"] == 0:
                        print('No object detected.')
                        return

                    # y,x centroid
                    image_coords = (M["m01"] / M["m00"], M["m10"] / M["m00"])

                    # "map" from color to depth image
                    depth_coords = (image_depth.shape[0]/2 + (image_coords[0] - camera_image.shape[0]/2)*self.color2depth_aspect, 
                        image_depth.shape[1]/2 + (image_coords[1] - camera_image.shape[1]/2)*self.color2depth_aspect)

                    # get the depth reading at the centroid location
                    depth_value = image_depth[int(depth_coords[0]), int(depth_coords[1])] # you might need to do some boundary checking first!

                    if np.isnan(depth_value):
                        continue

                    # Calculating the map co ordinate in 3d 
                    camera_coords = self.camera_model.projectPixelTo3dRay((image_coords[1], image_coords[0])) #project the image coords (x,y) into 3D ray in camera coords 
                    camera_coords = [x/camera_coords[2] for x in camera_coords] # adjust the resulting vector so that z = 1
                    camera_coords = [x*depth_value for x in camera_coords] # multiply the vector by depth

                    # object location
                    object_location = PoseStamped()
                    object_location.header.frame_id = "thorvald_001/kinect2_right_rgb_optical_frame"                
                    object_location.pose.position.x = camera_coords[0]
                    object_location.pose.position.y = camera_coords[1]
                    object_location.pose.position.z = camera_coords[2]
                    object_location.pose.orientation.w = 1.0

                    self.object_location_pub.publish(object_location)

                    p_camera = self.tf_listener.transformPose('map', object_location)
                    self.object_location.append([p_camera.pose.position.x, p_camera.pose.position.y, p_camera.pose.position.z] )
                    print('map coords: ', p_camera.pose.position)
                    print()

            #Module is gotten from https://scikit-learn.org/stable/modules/generated/sklearn.cluster.DBSCAN.html
            #eps is the maximum distance between two samples before one is no longer close. 
            epsilon = 0.09
            #min sample is the minimum number of samples that must be collected around a point before it may be deemed a core point in its neighbourhood. This can be subject to changed 
            minimum_sample = 4
            #DBSCAN is a clustering algorithm that defines clusters as continuous high-density zones, works well if all clusters are dense and well separated by low-density regions. 
            DB = DBSCAN(eps = epsilon, min_samples= minimum_sample).fit(self.object_location)
            DB_2 = DB.labels_
            Number_of_bunches = len(np.unique(DB_2))
            print('Number of grape bunches in the yard', Number_of_bunches)
            cv2.imshow("Region Of Interest", roi)
            cv2.waitKey(2)

        except:
            print()

def main():
    rospy.init_node("tracker", anonymous=True)
    Counter()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()