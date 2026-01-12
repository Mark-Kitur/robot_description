#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Image
import cv2
import cv_bridge
from rclpy.node import Node


class ViewImage(Node):
    def __init__(self):
        super().__init__("chess")

        self.brigde = cv_bridge.CvBridge()

        self.image_sub = self.create_subscription(
            Image,'/camera/image_raw',self.chess_callback,10
        )

        self.image_pub= self.create_publisher(
            Image,'/chess',10
        )

    def chess_callback(self,msg):
        try: 
            img = self.brigde.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            pub_img =self.brigde.cv2_to_imgmsg(img, encoding='bgr8')
            self.image_pub.publish(pub_img)

            cv2.imshow("chess",img)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"error in process img:{str(e)}")

def main(arfs=None):
    rclpy.init(args=arfs)
    node = ViewImage()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ =="__main__":
    main()
