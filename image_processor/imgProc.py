import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from imgproc_msgs.msg import Imgproc

import cv2
import numpy as np

class ImageProcessor(Node):

    def __init__(self):
        super().__init__('imageProcessor')
        self.publisher_ = self.create_publisher(Imgproc, 'img_data', 10)
        #self.publisher_ = self.create_publisher(String, 'img_data', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.cap = cv2.VideoCapture(0)

        self.marker_x = 0.0
        self.marker_y = 0.0
        self.marker_radius = 0.0

        self.p_marker_radius = 0.0

    def timer_callback(self):     
        '''

        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

       
        '''

        self._, self.frame = self.cap.read()
        self.frame = cv2.resize(self.frame, (int(self.frame.shape[1]/2), int(self.frame.shape[0]/2)))

        self.res_orange = self.getMask([0,50,100], [25,255,255])
        self.contours_frame = self.getContours(self.res_orange, 50, 30)

        # 再生
        #cv2.imshow('video',self.contours_frame)

        msg = Imgproc()

        msg.x = float(self.marker_x)
        msg.y = float(self.marker_y)

        if(self.p_marker_radius != self.marker_radius):
            msg.radius = float(self.marker_radius)
        else:
            msg.radius = 0.0

        self.p_marker_radius =  self.marker_radius
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.radius)
        
        



    def getMask(self, l, u):
        hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)

        lower = np.array(l)
        upper = np.array(u)

        if(lower[0] >= 0):
            mask = cv2.inRange(hsv, lower, upper)
        else:
            h = hsv[:, :, 0]
            s = hsv[:, :, 1]
            v = hsv[:, :, 2]
            mask = np.zeros(h.shape, dtype=np.uint8)
            mask[((h < lower[0]*-1) | h > upper[0]) & (s > lower[1]) & (s < upper[1]) & (v > lower[2]) & (v < upper[2])] = 255
        
        return cv2.bitwise_and(self.frame,self.frame, mask= mask)
    
    def getContours(self, img, t, r):
        gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        ret, thresh = cv2.threshold(gray, t, 255, cv2.THRESH_BINARY)

        contours, _= cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 一番大きい輪郭を抽出
        #contours.sort(key=cv2.contourArea, reverse=True)
 
        #一つ以上検出
        if len(contours) > 0:
            for cnt in contours:
                # 最小外接円を描く
                (x,y), radius = cv2.minEnclosingCircle(cnt)
                center = (int(x),int(y))
                radius = int(radius)

                
                #print("x:", x)
                #print("y:", y)
                #print("rad:", radius)

                radius_frame = cv2.circle(self.frame,center,0,(0,255,0),0)                

                if radius > r:
                    radius_frame = cv2.circle(self.frame,center,radius,(0,255,0),2)

                    self.marker_x = x
                    self.marker_y = y
                    self.marker_radius = radius

                    #print("radius:", radius)
                
                '''
                else:
                    self.marker_x = x
                    self.marker_y = y
                    self.marker_radius = 0.0
                    
                '''
                

            return radius_frame
        else:
            self.marker_radius = 0.0
            return self.frame

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()
        self.get_logger().info('Closing camera')


def main(args=None):
    rclpy.init(args=args)

    imgproc = ImageProcessor()

    rclpy.spin(imgproc)

    imgproc.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()