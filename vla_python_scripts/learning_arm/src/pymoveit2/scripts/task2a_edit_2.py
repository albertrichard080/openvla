#!/usr/bin/env python3






# Team ID:          [ CL#2202 ]
# Author List:		[ Amothini S, Jayanth G B, Deivaprakash K,  Vimal Grace M ]
# Filename:		    task2a.py
# Functions:
#			        [ main()   ]
# Nodes:		    Add your publishing and subscribing node
#                   Example:
#			        Publishing Topics   - [/servo_node/delta_twist_cmds,]
#                   Subscribing Topics  - [/tf]
#                   Service Clients     - [GripperMagnetON, GripperMagnetOFF, ]




#   Necessary Imports

from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
import math
import numpy as np
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from geometry_msgs.msg import TwistStamped
import tf_transformations
import time
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink
from pymoveit2 import MoveIt2Servo
from std_srvs.srv import Trigger
from std_msgs.msg import Int8



class PickAndPlace(Node):

    def __init__(self):
        super().__init__("pick_and_place")
        
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
           
        # self.callback_group_1 = ReentrantCallbackGroup()
        # self.callback_group_2 = ReentrantCallbackGroup()

        self.callback_group_1 = ReentrantCallbackGroup()
        self.callback_group_2 = None
        self.callback_group_3 = ReentrantCallbackGroup()
        self.callback_group_4 = ReentrantCallbackGroup()

        self.gripper_control_magnet_on = self.create_client(AttachLink,"GripperMagnetON")
        self.gripper_control_magnet_off = self.create_client(DetachLink,"GripperMagnetOFF")



        self.box_list_string = self.create_client(Trigger,'list_of_boxes',callback_group=self.callback_group_3)

        # self.servo_node = self.create_client(
        #     srv_type=Trigger,
        #     srv_name="/servo_node/start_servo",
        #     callback_group=self.callback_group_2,
        # )

        while not self.gripper_control_magnet_on.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GripperMagnetON service not available, waiting again...')

        while not self.gripper_control_magnet_off.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GripperMagnetOFF service not available, waiting again...')

        while not self.box_list_string.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('list_of_boxes service not available, waiting again...')

        # while not self.servo_node.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('/servo_node/start_servo service not available, waiting again...')

        self.create_subscription(Int8,"/servo_node/status",self.servo_status,10,callback_group=self.callback_group_4)





        self.picked_box_count = 0
        self.turned_right = False
        self.turned_left = False

        self.box_status = False
        self.box_not_found = True

        self.servo_status = 0

        
    # Create MoveIt 2 interface
        self.moveit2 = MoveIt2(
        node=self,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=self.callback_group_1,
        )

        #   Moveit2_servo Interface
        self.moveit2_servo = MoveIt2Servo(
            node = self,
            frame_id=ur5.base_link_name(),
            callback_group=self.callback_group_2
        )

        self.box_list = None
        # self.create_timer(1.0,self.find_boxes)


        
  
    def servo_status(self,data):
        self.servo_status = data.data
        
    
    def start_servo(self):
        '''
        Purpose:
        ---
        It is used to start the servo node

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.start_servo()
        '''

          
        if self.moveit2_servo.enable():
            self.get_logger().info("Servo enabled")
        else:
            self.get_logger().info("Servo cannot be enabled")

        # req = Trigger.Request()
        # future = self.servo_node.call_async(req)
        # rclpy.spin_until_future_complete(self,future)
        # response = future.result()
        # print(response)


       

    def find_boxes(self):
        '''
        Purpose:
        ---
        It is used to find the available boxes using tf. It is not used now. But will use in future.

        Input Arguments:
        ---
        None

        Returns:
        ---
        box_frames  :   list

        Example call:
        ---
        self.find_boxes()
        '''
        

        # for i in range(10):
        #     frames = self.tf_buffer.all_frames_as_string()
        #     rclpy.spin_once(self)
        
        # all_frames = [i.split()[1] for i in frames.split('\n') if len(i) != 0]
        # box_frames = filter(lambda x : True if "obj_" in x  else False,all_frames)
        # box_frames = list(box_frames)
        # self.get_logger().info(f"Available boxes are {box_frames}")
        print("Finding boxes")
        while ((not self.box_status) or (self.box_not_found)):
            req = Trigger.Request()
            future = self.box_list_string.call_async(req)
            # rclpy.spin_until_future_complete(self,future)
            # time.sleep(3)
            response = None
            while response is None:
                try:
                    response = future.result()
                except Exception as e:
                    print("Result not obtained waiting again")
        
            self.box_status = response.success
            box_list = response.message
            box_list = box_list.split()

            if not (len(box_list) == 0):
                self.box_not_found = False
            print(f"Finding boxes...")


        # executor = rclpy.executors.MultiThreadedExecutor(2)
        # executor.add_node(self)
        # executor_thread = Thread(target=executor.spin, daemon=True, args=())
        # executor_thread.start()

            
            

        c = 0
        for i in box_list:
            box_list[c] = f'obj_{i}'
            c += 1

        self.box_not_found = True
        self.box_status = False

       

        self.box_list = box_list
        print(self.box_list)

        # return box_list


    def find_position_of_box(self,box_name):
        '''
        Purpose:
        ---
        It is used to find the pose of the box (position and orientation)

        Input Arguments:
        ---
        box_name    :   string

        Returns:
        ---
        [translation, rotation]     :       [list(float),tuple(float)]

        Example call:
        ---
        self.find_position_of_box(obj_1)
        '''
        
        transform_stamped = None

        while transform_stamped is None:
            try:
                transform_stamped = self.tf_buffer.lookup_transform("base_link",box_name,rclpy.time.Time())
            except TransformException as e:
                self.get_logger().info(f"Can't able to transform from base_link to {box_name}")
            
            # rclpy.spin_once(self)
       
        translation = [transform_stamped.transform.translation.x,transform_stamped.transform.translation.y,transform_stamped.transform.translation.z]
        rotation_quaternion = [transform_stamped.transform.rotation.x,transform_stamped.transform.rotation.y,transform_stamped.transform.rotation.z,transform_stamped.transform.rotation.w]
        rotation = tf_transformations.euler_from_quaternion(rotation_quaternion)
        
        return [translation,rotation]
    
    def get_end_effector_current_pose(self):
        '''
        Purpose:
        ---
        It is used to find the end_effector current pose

        Input Arguments:
        ---
        None

        Returns:
        ---
        [translation, rotation]     :       [list(float),tuple(float)]

        Example call:
        ---
        self.get_end_effector_current_pose()
        '''

        transform_stamped = None

        while transform_stamped is None:

            try:
                transform_stamped = self.tf_buffer.lookup_transform("base_link","tool0",rclpy.time.Time())
            except TransformException as e:
                self.get_logger().info("Can't able to transform from base_link to tool0")

            # rclpy.spin_once(self)

        translation = [transform_stamped.transform.translation.x,transform_stamped.transform.translation.y,transform_stamped.transform.translation.z]
        rotation_quaternion = [transform_stamped.transform.rotation.x,transform_stamped.transform.rotation.y,transform_stamped.transform.rotation.z,transform_stamped.transform.rotation.w]
        rotation = tf_transformations.euler_from_quaternion(rotation_quaternion)

        return [translation,rotation]
    
    

    def go_to_box(self,box_name):
        '''
        Purpose:
        ---
        It is used to move the end effector to the specified box name

        Input Arguments:
        ---
        box_name    :   string

        Returns:
        ---
        None

        Example call:
        ---
        self.go_to_box(obj_1)
        '''


        position_box,rot_box = self.find_position_of_box(box_name)
        self.check_and_rotate_base(position_box[1])

        position_eef,orientation_eef = self.get_end_effector_current_pose()

        position_eef = np.array(position_eef)
        position_box = np.array(position_box)
        

        # distance_to_box = np.linalg.norm(position_eef-position_box)
        

        msg = TwistStamped()
        

        if self.turned_left or self.turned_right:
            position_box[1] = position_box[1] - 0.1
            distance_to_box = np.linalg.norm(position_eef-position_box)
            while (((position_box[0] - position_eef[0]) > 0.01) and ((position_box[2] - position_eef[2]) > 0.01)):

                # print(f"y distance = {position_box[1] - position_eef[1]}")
                msg.header.frame_id = "base_link"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.linear.x = (position_box[0] - position_eef[0]) 
                msg.twist.linear.y = (position_box[1] - position_eef[1])
                msg.twist.linear.z = (position_box[2] - position_eef[2]) 

                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0

                self.twist_pub.publish(msg)
                
                position_eef,orientation_eef = self.get_end_effector_current_pose()
                distance_to_box = np.linalg.norm(position_eef-position_box)

                # print(f"Remaining distance = {distance_to_box}")

            # print("Ready to pick")
            position_box[1] = position_box[1] + 0.1
            while (abs(position_box[1] - position_eef[1]) > 0.01):

                # print(f"y distance = {position_box[1] - position_eef[1]}")
                msg.header.frame_id = "base_link"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.linear.x = 0.0
                msg.twist.linear.y = (position_box[1] - position_eef[1])
                msg.twist.linear.z = 0.0

                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0

                self.twist_pub.publish(msg)
                
                position_eef,orientation_eef = self.get_end_effector_current_pose()
                distance_to_box = np.linalg.norm(position_eef-position_box)


                print(f"Remaining distance = {position_box[1] - position_eef[1]}")

        else:
            position_box[0] = position_box[0] - 0.1
            distance_to_box = np.linalg.norm(position_eef-position_box)

            while ((abs(position_box[1] - position_eef[1]) > 0.01) or (abs(position_box[2] - position_eef[2]) > 0.01)):
               
                # print(f"distance x = {position_box[0] - position_eef[0]}")
                msg.header.frame_id = "base_link"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.linear.x = (position_box[0] - position_eef[0])
                msg.twist.linear.y = (position_box[1] - position_eef[1])
                msg.twist.linear.z = (position_box[2] - position_eef[2])

                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0

                self.twist_pub.publish(msg)
                
                position_eef,orientation_eef = self.get_end_effector_current_pose()
                distance_to_box = np.linalg.norm(position_eef-position_box)

                print(f"Remaining distance = {abs(position_box[1] - position_eef[1])}, {abs(position_box[2] - position_eef[2])}")

            position_box[0] = position_box[0] + 0.1
            distance_to_box = np.linalg.norm(position_eef-position_box)

            while ((abs(position_box[0] - position_eef[0]) > 0.01)):
               
                # print(f"distance x = {position_box[0] - position_eef[0]}")
                msg.header.frame_id = "base_link"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.linear.x = (position_box[0] - position_eef[0])
                msg.twist.linear.y = 0.0
                msg.twist.linear.z = 0.0

                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0

                self.twist_pub.publish(msg)
                
                position_eef,orientation_eef = self.get_end_effector_current_pose()
                distance_to_box = np.linalg.norm(position_eef-position_box)

                print(f"Remaining distance = {position_box[0] - position_eef[0]}")
        # else:
        #     while ((distance_to_box > 0.02) and ((position_box[0] - position_eef[0]) > 0.001)):

        #         # print(f"distance x = {position_box[0] - position_eef[0]}")
        #         msg.header.frame_id = "base_link"
        #         msg.header.stamp = self.get_clock().now().to_msg()
        #         msg.twist.linear.x = (position_box[0] - position_eef[0])
        #         msg.twist.linear.y = (position_box[1] - position_eef[1])
        #         msg.twist.linear.z = (position_box[2] - position_eef[2])

        #         msg.twist.angular.x = 0.0
        #         msg.twist.angular.y = 0.0
        #         msg.twist.angular.z = 0.0

        #         self.twist_pub.publish(msg)
                
        #         position_eef,orientation_eef = self.get_end_effector_current_pose()
        #         distance_to_box = np.linalg.norm(position_eef-position_box)

        #         print(f"Remaining distance = {distance_to_box}")

            

   
        
           
        

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.twist_pub.publish(msg)

        print(f"Reached {box_name}")

    def move_up_litte(self):
        current_position_eef,current_orientation_eef = self.get_end_effector_current_pose()

        target_position_eef = [0,0,0]
        offset = 0.05
        target_position_eef[0] = current_position_eef[0] + offset
        target_position_eef[1] = current_position_eef[1] + offset
        target_position_eef[2] = current_position_eef[2] + offset

        msg = TwistStamped()

        print(f"Remaining up distance = {target_position_eef[2] - current_position_eef[2]}")
        
        while (abs(target_position_eef[2] - current_position_eef[2]) > 0.02):

            msg.header.frame_id = "base_link"
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = 0.0
            msg.twist.linear.y = 0.0
            msg.twist.linear.z = (target_position_eef[2] - current_position_eef[2])

            msg.twist.angular.x = 0.0
            msg.twist.angular.y = 0.0
            msg.twist.angular.z = 0.0

            self.twist_pub.publish(msg)
                
            current_position_eef,current_orientation_eef = self.get_end_effector_current_pose()
            

            print(f"Remaining up distance = {target_position_eef[2] - current_position_eef[2]}")

        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.twist_pub.publish(msg)



        

    def pick_box(self,box_name):
        '''
        Purpose:
        ---
        It is used to attach the box to end effector by calling /GripperMagnetON service

        Input Arguments:
        ---
        box_name    :   string

        Returns:
        ---
        None

        Example call:
        ---
        self.pick_box(obj_1)
        '''

        req = AttachLink.Request()
        box_name = box_name.strip("obj_")
        req.model1_name = f"box{box_name}"
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        self.gripper_control_magnet_on.call_async(req)
        self.picked_box_count += 1
        self.move_up_litte()
        self.come_back_little()

        if self.turned_left:
            self.turn_left_position()
        elif self.turned_right:
            self.turn_right_position()


    def place_box(self,box_name):
        '''
        Purpose:
        ---
        It is used to detach the box from the end effector by calling /GripperMagnetOFF service

        Input Arguments:
        ---
        box_name    :   string

        Returns:
        ---
        None

        Example call:
        ---
        self.place_box(obj_1)
        '''

        req = DetachLink.Request()
        box_name = box_name.strip("obj_")
        req.model1_name = f"box{box_name}"
        req.link1_name  = 'link'       
        req.model2_name =  'ur5'       
        req.link2_name  = 'wrist_3_link'  

        self.gripper_control_magnet_off.call_async(req)

    def come_back_little(self):
        '''
        Purpose:
        ---
        It is used to move the end effector little back for straight picking of object

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.come_back_little()
        '''

        msg = TwistStamped()
        position_eef,orientation_eef = self.get_end_effector_current_pose()

        tolerance = 0.25 # 0.22

        target_x = position_eef[0] - tolerance
        target_y = position_eef[1] - tolerance

        if self.turned_left:
            while ((abs(position_eef[1] - target_y) > 0.02) and (self.servo_status != 2)):

                msg.header.frame_id = "base_link"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.linear.x = 0.0
                msg.twist.linear.y = -(position_eef[1] - target_y)
               
                msg.twist.linear.z = 0.0

                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0

                self.twist_pub.publish(msg)
                # time.sleep(0.1)
                
                print("Going back little")
                position_eef,orientation_eef = self.get_end_effector_current_pose()


        elif self.turned_right:
            while ((abs(position_eef[1] - target_y) > 0.02) and (self.servo_status != 2)):

                msg.header.frame_id = "base_link"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.linear.x = 0.0
                msg.twist.linear.y = (position_eef[1] - target_y)
               
                msg.twist.linear.z = 0.0

                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0

                self.twist_pub.publish(msg)
                # time.sleep(0.1)
                
                print("Going back little")
                position_eef,orientation_eef = self.get_end_effector_current_pose()

        else:
            while ((abs(position_eef[0] - target_x) > 0.02) and (self.servo_status != 2)):

                msg.header.frame_id = "base_link"
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.linear.x = -(position_eef[0] - target_y)
                msg.twist.linear.y = 0.0
               
                msg.twist.linear.z = 0.0

                msg.twist.angular.x = 0.0
                msg.twist.angular.y = 0.0
                msg.twist.angular.z = 0.0

                self.twist_pub.publish(msg)
                # time.sleep(0.1)
                
                print("Going back little")
                position_eef,orientation_eef = self.get_end_effector_current_pose()



        

        # while (((position_eef[0] - target_x) > 0.02) or ((position_eef[1] - target_y) > 0.02)):

        #     msg.header.frame_id = "base_link"
        #     msg.header.stamp = self.get_clock().now().to_msg()

        #     if self.turned_left:
        #         msg.twist.linear.y = (position_eef[1] - target_y)
        #     elif self.turned_right:
        #         msg.twist.linear.y = -(position_eef[1] - target_y)
        #     else:
        #         msg.twist.linear.x = (position_eef[0] - target_x)
        #         msg.twist.linear.y = 0.0
            

        #     msg.twist.linear.z = 0.0

        #     msg.twist.angular.x = 0.0
        #     msg.twist.angular.y = 0.0
        #     msg.twist.angular.z = 0.0

        #     self.twist_pub.publish(msg)
        #     time.sleep(0.1)
            
        #     print("Going back little")

        
           
        
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.twist_pub.publish(msg)

        

    def pick_and_place_control(self,boxes_list):
        '''
        Purpose:
        ---
        It is main control function, where every method is used in this for pick and place action

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.pick_and_place_control()
        '''

        # boxes = self.box_list
        # boxes = self.find_boxes()
        
        # boxes = self.find_boxes()
        # box_positions = [self.find_position_of_box(box)[0][0] for box in boxes]
        # box_with_positions = [i for i in zip(boxes,box_positions)]
        # print(box_with_positions)
        # boxes_ordered_lengthwise = sorted(box_with_positions,key = lambda x : x[1],reverse = True)
        # boxes = [i for i,j in boxes_ordered_lengthwise]
        # print(boxes)
       
        
        # boxes = self.find_boxes()

        while self.box_list is None:
            print("Waiting for boxes")
      
        self.go_to_home_position()

        for box in self.box_list:
            self.go_to_box(box)
            self.pick_box(box)
            self.go_to_home_position()
            self.go_to_drop_position()
            self.place_box(box)
            self.go_to_home_position()
            

        self.get_logger().info("Task completed...")

    def go_to_home_position(self):
        '''
        Purpose:
        ---
        It is used to bring back the robot to home position

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.go_to_home_position()
        '''

        print("Moving to home position")

        if self.turned_right:
            self.turn_right_position()
            self.turned_right = False
        elif self.turned_left:
            self.turn_left_position()
            self.turned_left = False

    
        home_position = [-4.6256767574526236e-05, -2.390231027058905, 2.4004416219927935, -3.1501924424701175, -1.5799040814321978, 3.1500000117767297]
        self.moveit2.move_to_configuration(home_position)
        # time.sleep(7.0)
        self.moveit2.wait_until_executed()
 
        print("Reached Home position")


    def go_to_drop_position(self):
        '''
        Purpose:
        ---
        It is used to bring the robot to drop position

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.go_to_drop_position()
        '''

        print("Moving to drop position")

        if self.picked_box_count % 2 == 0:
            # drop_position = [0.00011561214616317983, -1.9183921870957872, -1.0473155151936324, -3.149930125021567, -1.5799971674061766, 3.1499267459762574]
            drop_position = [-0.00010523400787398884, -2.0943259439795674, -0.8711422315043751, -3.1500867325497914, -1.5800362372462775, 3.1499333575809816]
        else:
            # drop_position = [0.00011630800086326332, -1.742331217510611, -1.399169029405091, -3.1500165937023716, -1.5799051264690487, 3.1500664854954565]
            drop_position = [-0.0002539544884836431, -1.7424839005250656, -1.3991810536108562, -3.149197865225487, -1.5792088915800502, 3.149239453766056]

        self.moveit2.move_to_configuration(drop_position)
        # time.sleep(7.0)
        self.moveit2.wait_until_executed()

        print("Reached drop position")

       

    def turn_right_position(self):
        '''
        Purpose:
        ---
        It is used to turn the robot 90 degree right to avoid singularity as well as to pick the object in right

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.turn_right_position()
        '''

        print("Turning right position")

        right_position = [-1.5668058110446967, -2.3899325357900847, 2.3999901086739235, -3.150098364710264, -1.5799238441208967, 3.1499310123204802]
        self.moveit2.move_to_configuration(right_position)
        # time.sleep(7.0)
        self.moveit2.wait_until_executed()

        self.turned_right = True
        
        print("Turning right completed")

    def turn_left_position(self):
        '''
        Purpose:
        ---
        It is used to turn the robot 90 degree left to avoid singularity as well as to pick the object in left

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        self.turn_left_position()
        '''

        print("Turning left position")

        left_position = [1.5669041991747168, -2.390058608020905, 2.399998396708845, -3.1500253341730495, -1.5800506820004552, 3.1499210897933336]
        self.moveit2.move_to_configuration(left_position)
        # time.sleep(7.0)
        self.moveit2.wait_until_executed()

        self.turned_left = True
        print("Turning left completed")



    def check_and_rotate_base(self,y_distance):
        '''
        Purpose:
        ---
        It is used to trigger the turn_right_position() and turn_left_position() function based on y_distance

        Input Arguments:
        ---
        y_distance  :   float

        Returns:
        ---
        None

        Example call:
        ---
        self.check_and_rotate_base()
        '''

        if y_distance < -0.4:
            self.go_to_home_position()
            self.turn_right_position()
        elif y_distance > 0.4:
            self.go_to_home_position()
            self.turn_left_position()
            


def main():
    '''
        Purpose:
        ---
        It is used to instantiate object call the PickAndPlace methods

        Input Arguments:
        ---
        None

        Returns:
        ---
        None

        Example call:
        ---
        main()
        '''
    
    rclpy.init()
    node = PickAndPlace()
    node.start_servo()
    # boxes_list = node.find_boxes()
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()
 

    # node.turn_right_position()
    # rclpy.spin(node)
    
    node.pick_and_place_control("boxes_list")
    # node.find_boxes()
    # node.go_to_box('obj_1')
    # node.come_back_little()

    


    node.destroy_node()
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
