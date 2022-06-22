from submap_utils import *
from states import *
from actions import *
from debug_print import debugPrint
import numpy as np
import copy
import math
import rospy
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Twist
# Find color message type

class AirHockeyInterface:
    def __init__(self, robot_id, color):
        self.robot_id = robot_id
        self.color = color
        
        # Set air hockey interface parameters
        self.min_number_iterations = 5
        self.pos_error_thresh = 0.07 # m
        self.grid_to_vicon_conv_factor = 0.2 # m/grid cell
        
        # Set topic names based on robot_id
        waypoint_topic = "turtle" + str(robot_id + 1) + "/waypoint"
        grabber_topic = "turtle" + str(robot_id + 1) + "/electromag"
        led_topic = "turtle" + str(robot_id + 1) + "/color"
        pose_topic = "vicon/turtle" + str(robot_id + 1) + "/turtle" + str(robot_id + 1)
        # TODO: need topic for proximity sensor or somehting

        # Initialize ROS publishers and subscribers
        self.waypoint_pub = rospy.Publisher(waypoint_topic, Twist, queue_size=1, latch=True)
        self.grabber_pub = rospy.Publisher(grabber_topic, Bool, queue_size=1, latch=True)
        self.color_pub = rospy.Publisher(led_topic, ColorRGBA, queue_size=1, latch=True)
        self.pose_sub = rospy.Subscriber(pose_topic, TransformStamped, self.poseCallback)
        # TODO: need subscriber for proximity sensor or somehting
    
        # Set LED color
        self.color = color
        color_msg = ColorRGBA()
        color_msg.r = self.color[0]
        color_msg.g = self.color[1]
        color_msg.b = self.color[1]
        self.color_pub.publish(color_msg)

        self.loop_rate = rospy.Rate(10) # Hz
        self.true_pos_x = 0.0
        self.true_pos_y = 0.0


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
            if states.has_food == False and states.battery > 0: # Cannot grab if already posessing food or if battery is dead
                grabber_msg.data = True
                self.grabber_pub.publish(grabber_msg)
        elif action == Actions.DROP: # Drop food
            move_action = False
            drop_action = True
            grabber_msg = Bool()
            if states.has_food and states.battery > 0 and at_home:
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
            if i >= self.min_number_iterations and (pos_err * self.grid_to_vicon_conv_factor) < self.pos_error_thresh:
                debugPrint("goal reached")
                keep_executing = False
            self.loop_rate.sleep()

        # Observe outcome and produce new states
        new_states = copy.deepcopy(states)
        new_states.x = round(self.true_pos_x)
        new_states.y = round(self.true_pos_y)
        #new_states.has_food = self.food_sensor # TODO: make food sensor work

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
            # Check if food was found at the current location
            if isFoodAtPos(0, 0, submap):
                food_heading = getFoodHeading(0, 0, submap)
                # TEMP!!!!!!!!!!!!!!!!!!!!!
                new_states.has_food = True # TODO: for testing, remove this when food sensor is implemented
                new_states.food_heading = food_heading
                new_submap_object_list.append(MapLayer.FOOD)
                new_submap_property_list.append({"delta_x" : 0, "delta_y" : 0, "val" : 0}) # Remove food from robot's location on map
                # !!!!!!!!!!!!!!!!!!!!!!!!
        if drop_action:
            if states.has_food:
                # TEMP!!!!!!!!!!!!!!!!!!!!!
                new_states.has_food = False # TODO: for testing, remove this when food sensor is implemented
                new_states.food_heading = 0
                # !!!!!!!!!!!!!!!!!!!!!!!!

        # Update LED brightness based on battery charge
        color_msg = ColorRGBA()
        color_msg.r = self.color[0] * float(new_states.battery) / float(constants["battery_size"] - 1)
        color_msg.g = self.color[1] * float(new_states.battery) / float(constants["battery_size"] - 1)
        color_msg.b = self.color[1] * float(new_states.battery) / float(constants["battery_size"] - 1)
        self.color_pub.publish(color_msg)

        # Return new states and new submap
        new_submap = (new_submap_object_list, new_submap_property_list)
        return (new_states, new_submap)


    def poseCallback(self, msg):
        self.true_pos_x = msg.transform.translation.x / self.grid_to_vicon_conv_factor
        self.true_pos_y = msg.transform.translation.y / self.grid_to_vicon_conv_factor

    #def foodSensorCallback(self, msg):
    #    self.food_sensor = msg.data

    def isAtHome(self, x, y, home_pos):
        if x == home_pos[0] and y == home_pos[1]:
            return True
        else:
            return False
