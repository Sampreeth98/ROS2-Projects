import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import JointState
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from arduinobot_msgs.srv import EulerToQuaternion, QuaternionToEuler
from tf2_msgs.msg import TFMessage
import numpy as np
from arduinobot_msgs.msg import JointPub
from tf_transformations import quaternion_from_euler, euler_from_quaternion

class ImageSubscriber(Node):

    def __init__(self):
        super().__init__("image_subscriber")
        
        #self.endeff = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        # self.move_group = MoveGroupInterface("arm")
        self.currentstate = self.create_subscription(JointState, "/joint_states", self.storemsg, 10)
        self.imgRead = self.create_subscription(Image, "rgb_camera/image_raw", self.msgCallback, 10)
        self.jspublish = self.create_publisher(JointPub, "to_move_joints", 30)
        self.imgRead
        self.cv_bridge = CvBridge()
        self.model = YOLO("yolov8x.pt")
        self.prevjs = [0.0, 0.0, 0.0]
        self.tolerencepixels = 50.0

            
    def storemsg(self, msg):
        self.receivedpose = msg


    def msgCallback(self, msg):
        self.get_logger().info("Received an image message")
        try:
            self.roll2, self.pitch2, self.yaw2 = self.convertPose(self.receivedpose)
            jposition = self.receivedpose.position
            j1 = jposition[0]
            j2 = jposition[1]
            j3 = jposition[2]
            self.get_logger().info("joints j1: %f, j2: %f, j3: %f" % (j1, j2, j3))
            self.get_logger().info("Corresponding euler angles roll: %f, pitch: %f, yaw: %f" % (self.roll2, self.pitch2, self.yaw2))
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            detections = self.model(cv_image)
            new_image = cv_image.copy()
            # bbox = detections[0].boxes.xyxy
            # bbox_list = bbox.tolist() 
            for detection in detections:
                class_index = detection[0].boxes.cls

                # Retrieve bounding box coordinates
                bbox_tensor = detection[0].boxes.xyxy
                bbox_list = bbox_tensor.tolist()  # Convert tensor to a list of lists

                for bbox in bbox_list:
                    x_min, y_min, x_max, y_max = map(int, bbox)
                    center_point = [(x_max + x_min)/2, (y_max + y_min)/2]
                    current_center = [1152.0, 648.0]
                    move_required = [(center_point[0]-current_center[0]), (center_point[1]-current_center[1]) ]
                    # Draw bounding box on the new image
                    cv2.rectangle(new_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

                    # Put label
                    # label = "{}: {:.2f}".format(class_index, detection[0].boxes.conf)
                    # cv2.putText(new_image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Display the new image with detection results
            # cv2.imshow("Detection Result", new_image)
            # cv2.waitKey(0)  # Wait indefinitel
            self.get_logger().info("Current Center : {}".format(current_center))
            self.get_logger().info("Actual Center : {}".format(center_point))
            self.get_logger().info("Movement to be done : {}".format(move_required))
            if move_required == [0.0,0.0]:
                self.get_logger().info("Maintaining System ")
            
            
            # js1 = -0.0008 * move_required[0] + j1
            # js2 = 0.0 + j2
            # js3 = -0.0005 * move_required[1] + j3              
            if abs(move_required[0]) >= self.tolerencepixels and abs(move_required[1]) >= self.tolerencepixels:
                js1 = -0.0008 * move_required[0] + j1
                js2 = 0.0 + j2
                js3 = -0.0005 * move_required[1] + j3
            elif abs(move_required[0]) >= self.tolerencepixels and abs(move_required[1]) <= self.tolerencepixels:
                js1 = j1
                js2 = 0.0 + j2
                js3 = j3
            elif abs(move_required[0]) >= self.tolerencepixels and abs(move_required[1]) <= self.tolerencepixels :
                js1 = -0.0008 * move_required[0] + j1
                js2 = 0.0 + j2        
                js3 = j3
            else:
                js1 = j1
                js2 = 0.0 + j2
                js3 = -0.0005 * move_required[1] + j3
            
            
            jsp = JointPub()
            jsp.state_of_joint = [js1, js2, js3]
            self.get_logger().info("PREV JS : {}".format(self.prevjs))
            if self.prevjs == [0.0, 0.0, 0.0]:
                self.prevjs = [js1, js2, js3]
                self.jspublish.publish(jsp)
                self.get_logger().info("Joint states to move : {}".format(jsp.state_of_joint))
            else:
                if jsp.state_of_joint != self.prevjs:
                    self.prevjs = [js1, js2, js3]
                    self.jspublish.publish(jsp)
                    self.get_logger().info("Joint states to move : {}".format(jsp.state_of_joint))
                else:
                    self.prevjs = [js1, js2, js3]
            
                  
        except Exception as e:
            self.get_logger().error("Error processing image: {}".format(e))
            
    def convertPose(self, message):
        cposition = message.position
        j1 = cposition[0]
        j2 = cposition[1]
        j3 = cposition[2]
        j4 = cposition[3]
        roll1, pitch1, yaw1 = self.calculate_pose(j1, j2, j3, j4)
        # self.get_logger().info(str(j1))
        (x1, y1, z1, w1) = quaternion_from_euler(roll1, pitch1, yaw1)
        # self.get_logger().info("Corresponding euler angles roll: %f, pitch: %f, yaw: %f" % (roll1, pitch1, yaw1))
        # self.get_logger().info("Corresponding quaternion x: %f, y: %f, z: %f, w: %f" % (x1, y1, z1, w1))
        #(roll1, pitch1, yaw1) = euler_from_quaternion([req.x, req.y, req.z, req.w])
        return roll1, pitch1, yaw1
        #self.checkerror(x1,y1,z1,w1)    

    def calculate_pose(self, j1, j2, j3, j4):
        # DH Parameters
        d1 = 0.307
        a2 = -0.02
        a3 = 0.35
        d4 = 0.82
        
        # Convert angles to radians
        # j1 = np.radians(j1)
        # j2 = np.radians(j2)
        # j3 = np.radians(j3)
        # j4 = np.radians(j4)

        # Define DH parameter matrices
        # Transformation from frame 0 to frame 1 (Base link to Base plate)
        T01 = np.array([
            [np.cos(j1), -np.sin(j1), 0, 0],
            [np.sin(j1), np.cos(j1), 0, 0],
            [0, 0, 1, d1],
            [0, 0, 0, 1]
        ])

        # Transformation from frame 1 to frame 2 (Base plate to Forward drive arm)
        T12 = np.array([
            [np.cos(j2), 0, np.sin(j2), a2*np.cos(j2)],
            [0, 1, 0, 0],
            [-np.sin(j2), 0, np.cos(j2), a2*np.sin(j2)],
            [0, 0, 0, 1]
        ])

        # Transformation from frame 2 to frame 3 (Forward drive arm to Horizontal arm)
        T23 = np.array([
            [np.cos(j3), -np.sin(j3), 0, a3*np.cos(j3)],
            [np.sin(j3), np.cos(j3), 0, a3*np.sin(j3)],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # Transformation from frame 3 to frame 4 (Horizontal arm to Claw support)
        T34 = np.array([
            [np.cos(j4), 0, np.sin(j4), 0],
            [0, 1, 0, 0],
            [-np.sin(j4), 0, np.cos(j4), d4],
            [0, 0, 0, 1]
        ])

        # Combine transformations
        T04 = np.dot(np.dot(np.dot(T01, T12), T23), T34)

        # Extract roll, pitch, and yaw angles
        roll = np.arctan2(T04[2, 1], T04[2, 2])
        pitch = np.arctan2(-T04[2, 0], np.sqrt(T04[2, 1]**2 + T04[2, 2]**2))
        yaw = np.arctan2(T04[1, 0], T04[0, 0])

        return roll, pitch, yaw

def main():
    rclpy.init()

    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
