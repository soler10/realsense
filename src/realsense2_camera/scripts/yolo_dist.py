#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
import time

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError

import ros_numpy

from matplotlib import pyplot as plt
import math

import roboflow


# Load Yolo
rf = roboflow.Roboflow(api_key='iVQywCchrxMx0Dwu3VHu')

# Load a certain project (workspace url is optional)
project = rf.project("vineyards-uhs3r")
model = project.version("1").model
#colors
colors = np.random.uniform(0, 255, size=(10, 3))
font = cv2.FONT_HERSHEY_PLAIN

#Global variables
depth_image=np.zeros([1,1])
image=np.zeros([1,1])
pix=0




def callback_img(img):

	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(img, "bgr8") #results in numpy
	
	global image
	image=cv_image
	
	color_image = bridge.imgmsg_to_cv2(img, "bgr8") #results in numpy
	#print('im shape  ',color_image.shape)
	#print('  type color img   ',type(color_image))
	#cv2.imshow("Identificationfunction", color_image)
	result, objectInfo = getObjects(color_image)
	list_pub = rospy.Publisher('objectInfo', Float32MultiArray, queue_size=10)
	list_msg = Float32MultiArray()
	list_msg.data = objectInfo
	list_pub.publish(list_msg)
	#print(' image_shape ', cv_image.shape)
	
	
def callback_depth(data):
	
	int_data = list(pc2.read_points(data, field_names=('x','y','z'), skip_nans=False, uvs=[]))
	# Convert the generator to a NumPy array
	points = np.array(list(int_data))
	#print(points.shape)
	global depth_image
	depth_image=points[:,0:3]
	x=points[:,0]
	y=points[:,1]
	global z
	z=points[:,2]
		


def getObjects(img):
	
	#time.sleep(1)
	frame=img
	objectInfo = []
	prediction = model.predict(img)
	if len(prediction) != 0:
		print(prediction[0]["confidence"])
		class_ids = [ ]
		confidences = [ ]
		boxes = []
		
		for i in range(len(prediction)):
			class_id=prediction[i]["class"]
			class_ids.append(class_id)
			confidences.append(prediction[i]["confidence"])
			x=prediction[i]["x"]
			y=prediction[i]["y"]
			w=prediction[i]["width"]
			h=prediction[i]["height"]
			box=[x,y,w,h]
			x=int(x)
			y=int(y)
			w=int(w)
			h=int(h)
			print('bos int o que ', box)
			boxes.append(box)
			objectInfo.append([box,class_id])
			distance=get_dist(x,y)
			color=colors[i]
			start_point=(int(x-w/2), int(y-h/2))
			end_point=(int(x + w/2), int(y + h/2))
		
			cv2.rectangle(frame,start_point,end_point, color, 2)
			cv2.putText(frame, class_id , (x, y), font, 2, color, 2)
			cv2.putText(frame, "{}m".format(distance/1000), (x,y -30), cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255), 2)
			
			
		
	else:
		objectInfo=[0.0]
		print('no found')
	#objectInfo=prediction
		
		
	image_pub = rospy.Publisher("img_indentified",Image, queue_size=10)
	image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
	#cv2.imshow('aaaaaaaaaaaaaaaaaaaah', frame)
	#cv2.waitKey(0)
	return img, objectInfo		
	
def get_dist(center_x, center_y):
	x_im=image.shape[0]
	y_im=image.shape[1]
	x_d=depth_image.shape[0]
	y_d=depth_image.shape[1]
	print('im shape - ', x_im, ',', y_im, 'd shape -', x_d,',',y_d)
	x=int(center_x*x_d/x_im)
	y=int(center_y*y_d/y_im)
	print('center ', center_x, ' , ',center_y, ' punt depth ','x,y --> ', x, ' , ', y)
	#distance = depth_image[x][y]
	dist_prop=depth_image[y,x]
	d=cv2.resize(depth_image, dsize=(x_im, y_im), interpolation=cv2.INTER_CUBIC)
	distance=d[center_x,center_y]
	print('dist prop ', dist_prop,' dist resiize ', distance)
	return dist_prop
		
def callback_depth_2(img):
	bridge = CvBridge()
	cv_depth = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough") #results in numpy
	#print(cv_depth.shape)
	#print(cv_depth[240, 424])
	global depth_image
	depth_image=cv_depth
	

if __name__ == "__main__":
    # Realsense Camera Initialization
    #pipeline = rs.pipeline()
    #config = rs.config()
    #config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    #config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    #pipeline.start(config)
    #bridge = CvBridge()
    rospy.init_node('image_converter', anonymous=False)
    sub_img = rospy.Subscriber("/camera/color/image_raw",Image,callback_img)
    #time.sleep(1)
    #sub_depth = rospy.Subscriber("/camera/depth/color/points",PointCloud2,callback_depth)
    sub_depth = rospy.Subscriber("/camera/depth/image_rect_raw",Image,callback_depth_2)
    bridge = CvBridge()
    

    try:
        while True:
        	
            #time.sleep(0.5)
            #plt.imshow(img_depth, cmap='hot', interpolation='nearest')
            #print('img_depth   ', img_depth)
            #time.sleep(3)
            #plt.show()
            rospy.spin()
            #frames = pipeline.wait_for_frames()
            #color_frame = frames.get_color_frame()
            #color_image = np.asanyarray(color_frame.get_data())
            #cv2.imshow('RealSense', color_image)
            #depth_frame = frames.get_depth_frame()
            #print(type(depth_frame)) #class 'numpy.ndarray'
            #depth_image = np.asanyarray(depth_frame.get_data())
            #print(depth_image[0]) #class 'numpy.ndarray'
            #print(depth_frame)
            #result, objectInfo = getObjects(color_image)
            #print(objectInfo)
            #cv2.imshow('RealSense', color_image)
            #cv2.resizeWindow('RealSense', 800,800)
            
            #cv2.waitKey(1)

    except KeyboardInterrupt:
    		print("Shutting down")
    		cv2.destroyAllWindows()




