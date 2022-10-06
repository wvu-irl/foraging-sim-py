from submap_utils import *
from states import *
from actions import *
from debug_print import debugPrint
import numpy as np
import copy
import math
import re
import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist
# Find color message type

class AirHockeyInterface:
    def __init__(self, robot_id, num_food):
        self.robot_id = robot_id
        self.num_food = num_food
        
        # Set air hockey interface parameters
        self.steady_state_convergence_iterations = 10
        self.drive_goal_give_up_iterations = 50
        self.pos_error_thresh = 0.0254 # m
        self.grid_to_vicon_conv_factor = 0.05 # m/grid cell (0.2 for 10x5 map, 0.1 for 20x10 map, 0.05 for 40x20 map)
        
        self.true_pos_x = 0.0
        self.true_pos_y = 0.0
        self.food_visible = [False for i in range(self.num_food)]
        self.visible_food_pos_x = [0 for i in range(self.num_food)]
        self.visible_food_pos_y = [0 for i in range(self.num_food)]
        
        # Set topic names based on robot_id
        waypoint_topic = "turtle" + str(robot_id + 1) + "/waypoint"
        use_waypoint_topic = "turtle" + str(robot_id + 1) + "/use_waypoint"
        grabber_topic = "turtle" + str(robot_id + 1) + "/electromag"
        led_topic = "turtle" + str(robot_id + 1) + "/color"
        pose_topic = "vicon/turtle" + str(robot_id + 1) + "/turtle" + str(robot_id + 1)
        force_sensor_topic = "turtle" + str(robot_id + 1) + "/force_sensor"
        food_topics = ["vicon/food" + str(i) + "/food" + str(i) for i in range(1, num_food+1)]

        # Initialize ROS publishers and subscribers
        self.waypoint_pub = rospy.Publisher(waypoint_topic, Twist, queue_size=1, latch=True)
        self.use_waypoint_pub = rospy.Publisher(use_waypoint_topic, Bool, queue_size=1, latch=True)
        self.grabber_pub = rospy.Publisher(grabber_topic, Bool, queue_size=1, latch=True)
        self.color_pub = rospy.Publisher(led_topic, ColorRGBA, queue_size=1, latch=True)
        self.pose_sub = rospy.Subscriber(pose_topic, TransformStamped, self.poseCallback)
        self.force_sensor_sub = rospy.Subscriber(force_sensor_topic, Bool, self.forceSensorCallback)
        self.food_subs = [rospy.Subscriber(food_topics[i], TransformStamped, self.foodPosCallback) for i in range(num_food)]
        
        self.loop_rate = rospy.Rate(10) # Hz

    def sendInitCmd(self, init_x, init_y):
        grabber_msg = Bool()
        grabber_msg.data = False
        self.grabber_pub.publish(grabber_msg)
        goal_msg = Twist()
        goal_msg.linear.x = float(init_x) * self.grid_to_vicon_conv_factor
        goal_msg.linear.y = float(init_y) * self.grid_to_vicon_conv_factor
        self.waypoint_pub.publish(goal_msg)
        use_waypoint_msg = Bool()
        use_waypoint_msg.data = True
        self.use_waypoint_pub.publish(use_waypoint_msg)

    def sendStopCmd(self):
        use_waypoint_msg = Bool()
        use_waypoint_msg.data = False
        self.use_waypoint_pub.publish(use_waypoint_msg)

    def resetVisibleFood(self):
        self.food_visible = [False for i in range(self.num_food)]

    def executeTransition(self, states, submap, action, constants):
        new_submap_object_list = []
        new_submap_property_list = []
        # Record initial values from current states
        initial_x = states.x
        initial_y = states.y
        map_shape = constants["map_shape"]
        home_pos = constants["home_pos"]
        at_home = self.isAtHome(states.x, states.y, home_pos)

        # Send commands based on chosen action
        grab_action = False
        drop_action = False
        if action == Actions.STAY: # Stay
            (delta_x, delta_y) = getDeltaFromDirection(Direction.NONE)
            move_action = False
        elif action == Actions.MOVE_E: # Move E
            (delta_x, delta_y) = getDeltaFromDirection(Direction.E)
            move_action = True
        elif action == Actions.MOVE_NE: # Move NE
            (delta_x, delta_y) = getDeltaFromDirection(Direction.NE)
            move_action = True
        elif action == Actions.MOVE_N: # Move N
            (delta_x, delta_y) = getDeltaFromDirection(Direction.N)
            move_action = True
        elif action == Actions.MOVE_NW: # Move NW
            (delta_x, delta_y) = getDeltaFromDirection(Direction.NW)
            move_action = True
        elif action == Actions.MOVE_W: # Move W
            (delta_x, delta_y) = getDeltaFromDirection(Direction.W)
            move_action = True
        elif action == Actions.MOVE_SW: # Move SW
            (delta_x, delta_y) = getDeltaFromDirection(Direction.SW)
            move_action = True
        elif action == Actions.MOVE_S: # Move S
            (delta_x, delta_y) = getDeltaFromDirection(Direction.S)
            move_action = True
        elif action == Actions.MOVE_SE: # Move SE
            (delta_x, delta_y) = getDeltaFromDirection(Direction.SE)
            move_action = True
        elif action == Actions.GRAB: # Grab food
            move_action = False
            grab_action = True
            grabber_msg = Bool()
            if states.battery > 0: # Cannot grab if already posessing food or if battery is dead
                grabber_msg.data = True
                self.grabber_pub.publish(grabber_msg)
        elif action == Actions.DROP: # Drop food
            move_action = False
            drop_action = True
            grabber_msg = Bool()
            if states.battery > 0:
                grabber_msg.data = False
                self.grabber_pub.publish(grabber_msg)
        else:
            raise RuntimeError("action is not valid: {0}".format(action))

        if move_action:
            goal_x = float(initial_x + delta_x)
            goal_y = float(initial_y + delta_y)
            if (0 <= int(goal_x) < map_shape[0]) and (0 <= int(goal_y) < map_shape[1]) and states.battery > 0: # Only send the drive goal if it is within the map boundaries and battery not dead
                goal_msg = Twist()
                goal_msg.linear.x = goal_x * self.grid_to_vicon_conv_factor
                goal_msg.linear.y = goal_y * self.grid_to_vicon_conv_factor
                self.waypoint_pub.publish(goal_msg)
            else:
                goal_x = float(initial_x)
                goal_y = float(initial_y)
        else:
            goal_x = float(initial_x)
            goal_y = float(initial_y)

        # Perform chosen action
        keep_executing = True
        i = 0
        while (not rospy.is_shutdown()) and keep_executing:
            x_err = goal_x - self.true_pos_x # grid cells
            y_err = goal_y - self.true_pos_y # grid cells
            pos_err = math.hypot(x_err, y_err)
            debugPrint("i: {0}, pos_err: {1}".format(i, pos_err * self.grid_to_vicon_conv_factor))
            i += 1
            if i >= self.steady_state_convergence_iterations and (pos_err * self.grid_to_vicon_conv_factor) < self.pos_error_thresh:
                debugPrint("goal reached")
                keep_executing = False
            elif i >= self.drive_goal_give_up_iterations: # If taking too long to reach goal (i.e., robot stuck) give up and send goal to current position to stop trying to drive
                goal_msg = Twist()
                goal_msg.linear.x = round(self.true_pos_x) * self.grid_to_vicon_conv_factor
                goal_msg.linear.y = round(self.true_pos_y) * self.grid_to_vicon_conv_factor
                self.waypoint_pub.publish(goal_msg)
                keep_executing = False
            self.loop_rate.sleep()

        # Observe outcome and produce new states
        new_states = copy.deepcopy(states)
        new_states.x = round(self.true_pos_x)
        new_states.y = round(self.true_pos_y)
        new_states.has_food = self.food_sensor

        # If robot has moved, mark old position in map to be removed and increment distance traversed
        if new_states.x != initial_x or new_states.y != initial_y:
            new_submap_object_list.append(MapLayer.ROBOT)
            new_submap_property_list.append({"delta_x" : -delta_x, "delta_y" : -delta_y, "id" : constants["id"]})

        # If at home, battery receives charge
        new_at_home = self.isAtHome(new_states.x, new_states.y, home_pos)
        if new_at_home:
            new_states.battery = constants["battery_size"] - 1
        else:
            if move_action:
                new_states.battery = states.battery - 1
                if new_states.battery < 0:
                    new_states.battery = 0
        if grab_action: 
            keep_executing = True
            i = 0
            while (not rospy.is_shutdown()) and keep_executing:
                debugPrint("i: {0}, food_sensor: {1}".format(i, self.food_sensor))
                i += 1
                if i >= self.steady_state_convergence_iterations:
                    keep_executing = False
                    new_states.has_food = self.food_sensor
                    if self.food_sensor:
                        new_submap_object_list.append(MapLayer.FOOD)
                        new_submap_property_list.append({"delta_x" : 0, "delta_y" : 0, "val" : 0}) # Remove food from robot's location on map
                self.loop_rate.sleep()
            if not new_states.has_food:
                grabber_msg = Bool()
                grabber_msg.data = False                
                self.grabber_pub.publish(grabber_msg)
        if drop_action:
            keep_executing = True
            i = 0
            while (not rospy.is_shutdown()) and keep_executing:
                debugPrint("i: {0}, food_sensor: {1}".format(i, self.food_sensor))
                i += 1
                if i >= self.steady_state_convergence_iterations:
                    keep_executing = False
                    new_states.has_food = self.food_sensor
                self.loop_rate.sleep()

        # Update LED color based on heading and brightness based on battery charge
        color_msg = ColorRGBA()
        if new_states.heading in [0, 1]:
            color = [255, 0, 0]
        elif new_states.heading == 3:
            color = [0, 0, 255]
        elif new_states.heading == 5:
            color = [255, 0, 255]
        color_msg.r = color[0] * float(new_states.battery) / float(constants["battery_size"] - 1)
        color_msg.g = color[1] * float(new_states.battery) / float(constants["battery_size"] - 1)
        color_msg.b = color[1] * float(new_states.battery) / float(constants["battery_size"] - 1)
        #self.color_pub.publish(color_msg) # TODO: fix LEDs on robot and uncomment

        # Return new states and new submap
        new_submap = (new_submap_object_list, new_submap_property_list)
        return (new_states, new_submap)


    def poseCallback(self, msg):
        self.true_pos_x = msg.transform.translation.x / self.grid_to_vicon_conv_factor
        self.true_pos_y = msg.transform.translation.y / self.grid_to_vicon_conv_factor

    def forceSensorCallback(self, msg):
        self.food_sensor = msg.data

    def foodPosCallback(self, msg):
        # regex match to extract number from msg.child_frame_id to determine which food it is
        food_index = int(re.match('.*?(\d+)$', msg.child_frame_id).group(1)) - 1
        self.food_visible[food_index] = True
        self.visible_food_pos_x[food_index] = round(msg.transform.translation.x / self.grid_to_vicon_conv_factor)
        self.visible_food_pos_y[food_index] = round(msg.transform.translation.y / self.grid_to_vicon_conv_factor)

    def isAtHome(self, x, y, home_pos):
        if x == home_pos[0] and y == home_pos[1]:
            return True
        else:
            return False
