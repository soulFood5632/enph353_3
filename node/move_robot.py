#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np



class CarController:
    k_p = 0.03
    k_d = 0.0
    k_i = 0.0

    speed = 0.4
    slow_speed = 0.4


    maxError = 400


    def __init__(self):
        rospy.init_node('controller', anonymous=True)
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.bridge = CvBridge()
        self.subsriber = rospy.Subscriber('rrbot/camera1/image_raw', Image, callback=self.__handleImage)
        self.loopRate = rospy.Rate(10)
        self.last_error = 0
        self.total_error = 0

    def listen(self):
        rospy.loginfo('ready to listen')
        rospy.spin()
        
        
    def __handleImage(self, cameraImage):
        cvImage = self.bridge.imgmsg_to_cv2(cameraImage)
        error = self.__findError(cvImage)

        rospy.loginfo("firstError = %i", error)

        error = self.__applyClipping(error=error)

        rospy.loginfo("%i", error)

        angularVel = self.__steering_command(error=error)

        move = Twist()
        move.angular.z = angularVel
        speed = self.speed
        if np.abs(angularVel) > 0.8:
            speed = self.slow_speed
        move.linear.x = speed

        # rospy.loginfo('%i', angularVel)

        self.publisher.publish(move) 
        # didn't think this line was needed based on current assesments of the speed of the control loop. 

        # self.loopRate.sleep()



    def __applyClipping(self, error: int) -> int:
        # if np.abs(error) > 50:
        #     rospy.loginfo('exit max turn state')
        #     self.isMaxLeft = False
        #     self.isMaxRight = False
        
        # if np.abs(error) < 50:
        #     if abs(self.last_error) > 75:
        #         if self.last_error < 0:
        #             rospy.loginfo('max left turn')
        #             self.isMaxLeft = True
        #         else:
        #             rospy.loginfo('max right turn')
        #             self.isMaxRight = True


        
        # if self.isMaxLeft:
        #     error = -self.maxError
        # if self.isMaxRight:
        #     error = self.maxError

        return error






    def __steering_command(self, error: int) -> float:
        derivative_error = error - self.last_error
        self.total_error += error
        self.last_error = error

        return self.k_p * error + self.k_d * derivative_error + self.k_i * self.total_error
        


    def __findError(self, cvImg) -> int:
        gray = cv2.cvtColor(cvImg, cv2.COLOR_BGR2GRAY)

        blurred = cv2.blur(gray, (7, 7), 0)

        height, width, _ = cvImg.shape
        bottom_percentage = 0.32  # Adjust as needed
        crop_height = int(height * bottom_percentage)

        cropped_image = blurred[height - crop_height:, :]

        _, thresholded = cv2.threshold(cropped_image, 128, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        min_blob_area = 50  # Adjust as needed
        filtered_contours = [contour for contour in contours if cv2.contourArea(contour) >= min_blob_area]

        if filtered_contours:
            largest_contour = max(filtered_contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                largest_centroid_x = int(M["m10"] / M["m00"])
            
            else:
                largest_centroid_x = 0
        else:
            largest_centroid_x = width / 2

        # rospy.loginfo('centroid: %i, width: %i', largest_centroid_x, width / 2)

                    
        return int(largest_centroid_x - width / 2)



if __name__ == '__main__':
    carController = CarController()
    carController.listen()
    


        







# def listener():
   
#     rospy.init_node('controller', anonymous=True) # anonomous will help ensure that if new listener are created it does not overwrite this current one
#     rospy.Subscriber('rrbot/camera1/image_raw', )
#     rospy.spin()


# while not rospy.is_shutdown():
#     pub.publish(move)
#     rate.sleep()