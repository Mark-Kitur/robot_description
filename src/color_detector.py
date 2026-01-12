#!/usr/bin/env python3

import rclpy, cv2, tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from rclpy.duration import Duration
from transforms3d.quaternions import quat2mat


def quat_matrix(q):
    x,y,z,w =q
    r= np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),         1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [2*(x*z - y*w),         2*(y*z + x*w),       1 - 2*(x*x + y*y)]
    ])
    # r= np.eye(4)
    # r[0:3,0:3] =r
    return r

class ColorDetector(Node):
    def __init__(self):
        super().__init__("color_detector_node")
        self.width =640
        self.height= 480
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer,self)

        self.ouput_pub= self.create_publisher(
            String,
            'detector',  # Fixed typo: 'detecor' -> 'detector'
            10
        )
        self.image_sub=self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        ) 
        self.image_pub= self.create_publisher(Image,'/detected_image',10)

        self.bridge= CvBridge()

        self.color_ranges = {
            'red': [
                # Red wraps around in HSV, so we need two ranges
                (np.array([0, 100, 100]), np.array([10, 255, 255])),
                (np.array([160, 100, 100]), np.array([180, 255, 255]))
            ],
            'green': [
                (np.array([40, 50, 50]), np.array([80, 255, 255]))
            ],
            'blue': [
                (np.array([90, 50, 50]), np.array([130, 255, 255]))
            ],
            'yellow': [
                (np.array([20, 100, 100]), np.array([40, 255, 255]))
            ]
        }

        self.get_logger().info('color box detector has started')

    def detect_color(self, hsv_image, color_name):
        masks=[]

        for lower, upper in self.color_ranges[color_name]:  # Fixed: removed extra comma
            mask = cv2.inRange(hsv_image, lower, upper)
            masks.append(mask)  # Fixed: added 'mask' argument

        combined_mask = masks[0]
        for mask in masks[1:]:
            combined_mask = cv2.bitwise_or(combined_mask, mask)
        
        return combined_mask
    
    def find_contoures(self, mask, min_area=500):
        kernel= np.ones((5,5),np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,kernel)

        contours,_ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        filtered_counters = [cnt for cnt in contours if cv2.contourArea(cnt)>min_area]

        return contours
    
    def image_callback(self,msg):
        '''process image'''
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            result_image = cv_image.copy()

            detected_colors=[]

            for color_name in self.color_ranges.keys():
                mask = self.detect_color(hsv_image, color_name)
                contours = self.find_contoures(mask)

                for contour in contours:
                    x,y,w,h = cv2.boundingRect(contour)

                    center_x = x+w//2
                    center_y = y+h //2

                    draw_colors= {
                        'red':(0,0,255),
                        'green':(0, 255,0),
                        'blue':(255,0,0),
                        'yellow':(0,255,255)
                    }
                    cv2.circle(result_image,(int(self.width/2) ,int(self.height/2)),20,(100,255,100),-1)

                    cv2.rectangle(result_image,(x,y), (x+w,y+h), draw_colors[color_name],2)

                    label = f"{color_name.upper()}"
                    cv2.putText(result_image, label, (x,y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_colors[color_name],2)

                    cv2.circle(result_image,(center_x,center_y),5, (255,255,255),-1)

                    detected_colors.append({
                        'color':color_name,
                        'position':(center_x,center_y),
                        'area':cv2.contourArea(contour)
                    })
                    self.get_logger().info(f'Detected {color_name} box at ({center_x}, {center_y})')
                    # project pixel -> camera to 3D coordinates
                    fx =9.68422187e+03
                    fy= 9.66837504e+03
                    cx= 3.68205375e+02
                    cy= 4.46852641e+01

                    Z= 1
                    Y= (center_x - cx)*Z /fx *-10
                    X= (center_y - cy)*Z /fy



                    t = self.tf_buffer.lookup_transform(
                        'base_link', 'camera_link', msg.header.stamp,timeout=Duration(seconds=0.5)
                    )
                    rot = [
                        t.transform.rotation.x,
                        t.transform.rotation.y,
                        t.transform.rotation.z,
                        t.transform.rotation.w
                    ]
                    
                    trans = np.array([
                        t.transform.translation.x,
                        t.transform.translation.y,
                        t.transform.translation.z,
                    ])
                    T= quat_matrix(rot)
                    #T = np.eye(4)
                    # T[0:3, 0:3] = R
                    T[0:3, 2] = trans

                    pt_cam = np.array([X, Y, Z])
                    P = T @ pt_cam
                                        
                    coords = f"{color_name}, {P[0]:.2f}, {P[1]:.2f}, {P[2]:.2f}"
                    self.get_logger().info(coords)

            if detected_colors:
                detection_msg=String()
                detection_msg.data =', '.join([d['color'] for d in detected_colors])
                self.ouput_pub.publish(detection_msg)  

            result_msg = self.bridge.cv2_to_imgmsg(result_image,encoding='bgr8')
            self.image_pub.publish(result_msg)

            cv2.imshow("color Detection", result_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error in processing image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()  
    cv2.destroyAllWindows()

if __name__ =="__main__":
    main()