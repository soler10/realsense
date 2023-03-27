#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import torch
import rospy
import numpy as np
import time

from std_msgs.msg import Header
from sensor_msgs.msg import Image
from yolov5_ros_msgs.msg import BoundingBox, BoundingBoxes

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2

from cv_bridge import CvBridge, CvBridgeError


#parametres d435:
#focal length
fx=636.4285888671875
fy=635.8297119140625
#principal point
ppx=636.2267456054688
ppy=378.491455078125

class Pointer:
    def __init__(self):

        # load parameters
        image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        depth_topic=rospy.get_param('~depth_topic', '/camera/depth/image_rect_raw')
        pointcloud_topic=rospy.get_param('~pointcloud_topic', '/voxel_cloud')
        position_topic = rospy.get_param('~position_topic', '/yolov5/BoundingBoxes')

        
        self.camera_frame = rospy.get_param('~camera_frame', '')
        conf = rospy.get_param('~conf', '0.5')

        self.color_image = Image()
        self.depth_image = Image()
        self.depth_image_pc = Image()
        self.z=[]
        self.getImageStatus = False
        self.distance=-1.0

        # Load class color
        self.classes_colors = {}

        # image subscribe
        self.color_sub = rospy.Subscriber(image_topic, Image, self.image_callback, queue_size=1, buff_size=52428800)

        # depth img subscribe
        self.depth_sub = rospy.Subscriber(depth_topic, Image, self.depth_callback,queue_size=1, buff_size=52428800)

        #point cloud subscriber
        self.pointcloud_sub = rospy.Subscriber(pointcloud_topic, PointCloud2, self.pointcloud_callback,queue_size=1, buff_size=52428800)

        # bounding boxes subscriber
        self.position_sub = rospy.Subscriber(position_topic,  BoundingBoxes, self.position_callback, queue_size=1, buff_size=52428800)

        #publish image
        self.image_pub = rospy.Publisher('/yolov5/detection_image',  Image, queue_size=1)

        # if no image messages
        while (not self.getImageStatus) :
            rospy.loginfo("waiting for image.")
            rospy.sleep(2)


    def image_callback(self, image):
        self.getImageStatus = True
        self.color_image = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.width, -1)
        self.color_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2RGB)

    def depth_callback(self, image):
        bridge = CvBridge()
        self.depth_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough") #results in numpy
	

    

    def pointcloud_callback(self ,data):
        #print("PC CALLBACK")
        int_data = list(pc2.read_points(data, field_names=('x','y','z'), skip_nans=False, uvs=[]))
        # Convert the generator to a NumPy array
        points = np.array(list(int_data))
        #print(points.shape)
        d=data
        self.depth_image_pc=points[:,0:3]
        x=points[:,0]
        y=points[:,1]
        self.z=points[:,2]

    def position_callback(self, data):
        #print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        bb=data.bounding_boxes
        #print('x ->',bb[0])
        for i in range(len(bb)):
            box=[bb[i].xmin, bb[i].ymin ,bb[i].xmax ,bb[i].ymax, 0, bb[i].Class] 
            xmin=bb[i].xmin
            ymin=bb[i].ymin
            xmax=bb[i].xmax
            ymax=bb[i].ymax
            label=bb[i].Class
            #print('xmin ymin xmax ymax class -->',xmin,' ',ymin,' ', xmax,' ',ymax,' ',label)
            mig=[int((xmax-xmin)/2+xmin), int((ymax-ymin)/2+ymin)]
            #print("image shape o algoooooooooooo ", len(self.color_image[0]))
            #self.dectshow(self.color_image, box, self.color_image.shape, len(self.color_image[0]))
            #print("punt mig ", mig)
            distance=self.get_dist(mig[0],mig[1])
            self.proj_point(mig[0],mig[1], distance)
            
    
        #print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
    #Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, compute the corresponding point in 3D space relative to the same camera 
    def proj_point(self, px,py, depth):
        x=(px-ppx*depth)/fx
        y=(py-ppy*depth)/fy
        z=depth
        time.sleep(1)
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = 'camera_link'
        point_msg.point.x =x #point[0]
        point_msg.point.y =y # point[1]
        point_msg.point.z =depth # point[2]
        print('pixel ', px, ' ',py, ' x i y i z ',x,' ',y ,' ', z)
        publisher = rospy.Publisher('/cep_found', PointStamped, queue_size=10)
        publisher.publish(point_msg)
        
    def get_dist(self, center_x, center_y):
        x_im=self.color_image.shape[0]
        #print(' linia 121    ', self.depth_image.shape)
        y_im=self.color_image.shape[1]
        x_d=self.depth_image.shape[0]
        y_d=self.depth_image.shape[1]
        #print('im shape - ', x_im, ',', y_im, 'd shape -', x_d,',',y_d)
        x=int(center_x*x_d/x_im)
        y=int(center_y*y_d/y_im)
        #print('center ', center_x, ' , ',center_y, ' punt depth ','x,y --> ', x, ' , ', y)
        #distance = depth_image[x][y]
        dist_prop=self.depth_image[y][x]
        #print('         ',self.depth_image[1][1] )
        d=cv2.resize(self.depth_image, dsize=(x_im, y_im), interpolation=cv2.INTER_CUBIC)
        distance=d[center_x,center_y]
        #print('dist prop ', dist_prop,' dist resiize ', distance)
        return dist_prop

    def dectshow(self, org_img, box, height, width):
        img = org_img.copy()

        if box[-1] in self.classes_colors.keys():
            color = self.classes_colors[box[-1]]
        else:
            color = np.random.randint(0, 183, 3)
            self.classes_colors[box[-1]] = color

        cv2.rectangle(img, (int(box[0]), int(box[1])),
                        (int(box[2]), int(box[3])), (int(color[0]),int(color[1]), int(color[2])), 2)

        if box[1] < 20:
            text_pos_y = box[1] + 30
        else:
            text_pos_y = box[1] - 10
        print(box[-1]+' aaaaa'+ str(self.distance))    
        cv2.putText(img, box[-1],
                    (int(box[0]), int(text_pos_y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

         #print('aaaaaaaaah ',self.boundingBoxes) 
        self.publish_image(img, height, width)
        #cv2.imshow('YOLOv5', img)
        #cv2.waitKey(0)
    

    def publish_image(self, imgdata, height, width):
        image_temp = Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = self.camera_frame
        image_temp.height = height
        image_temp.width = width
        image_temp.encoding = 'bgr8'
        image_temp.data = np.array(imgdata).tobytes()
        image_temp.header = header
        image_temp.step = width * 3
        self.image_pub.publish(image_temp)


def main():
    rospy.init_node('pointer_ros', anonymous=True)
    pointer = Pointer()
    rospy.spin()


if __name__ == "__main__":

    main()
