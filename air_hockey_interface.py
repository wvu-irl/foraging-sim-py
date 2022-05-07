from submap_utils import *
from states import *
from actions import *
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
        
        # Set topic names based on robot_id
        waypoint_topic = "turtle" + str(robot_id + 1) + "/waypoint"
        grabber_topic = "turtle" + str(robot_id + 1) + "/electromag"
        led_topic = "turtle" + str(robot_id + 1) + "/color"
        pose_topic = "vicon/turtle" + str(robot_id + 1) + "/turtle" + str(robot_id + 1)
        # TODO: need topic for proximity sensor or somehting

        # Initialize ROS publishers and subscribers
        self.waypoint_pub = rospy.Publisher(waypoint_topic, Twist, queue_size=1)
        self.grabber_pub = rospy.Publisher(grabber_topic, Bool, queue_size=1)
        self.color_pub = rospy.Publisher(led_topic, SOMETHING, queue_size=1)
        self.pose_sub = rospy.Subscriber(pose_topic, TransformStamped, self.poseCallback)
        # TODO: need subscriber for proximity sensor or somehting
    
        # Set LED color
        color_msg = ColorRGBA
        color_msg.r = color["r"]
        color_msg.g = color["g"]
        color_msg.b = color["b"]
        self.color_pub.publish(color_msg)

        self.loop_rate = rospy.Rate(10) # Hz

        self.min_number_iterations = 5
        self.pos_error_thresh = 0.1 # m

    def executeTransition(self, states, action, constants):
        # Record initial values from current states
        initial_x = states.x
        initial_y = states.y
        map_shape = constants["map_shape"]
        home_pos = constants["home_pos"]
        num_food = constants["num_food"]
        food_pos = constants["food_pos"]
        food_heading = constants["food_heading"]
        food_cluster = constants["food_cluster"]
        food_map = getFoodMapFromBinary(states.food_state, num_food, food_pos, map_shape)
        at_home = isAtHome(states.x, states.y, home_pos)

        # Send commands based on chosen action
        grab_action = False
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
            if states.has_food == False and states.battery > 0: # Cannot grab if already posessing food or if battery is dead
                grabber_msg = Bool
                grabber_msg.data = True
                grabber_pub.publish(grabber_msg)
        elif action == Actions.DROP: # Drop food
            move_action = False
            if states.has_food and states.battery > 0 and at_home:
                grabber_msg = Bool
                grabber_msg.data = False
                grabber_pub.publish(grabber_msg)
        else:
            raise RuntimeError("action is not valid: {0}".format(action))

        if move_action:
            goal_x = float(initial_x + delta_x)
            goal_y = float(initial_y + delta_y)
            if (0 <= int(goal_x) < map_shape[0]) and (0 <= int(goal_y) < map_shape[1]) and states.battery > 0: # Only send the drive goal if it is within the map boundaries and battery not dead
                goal_msg = Twist
                goal_msg.linear.x
                goal_msg.linear.y
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
            x_err = goal_x - self.true_pos_x
            y_err = goal_y - self.true_pos_y
            pos_err = math.hypot(x_err, y_err)
            i += 1
            if i >= self.min_number_iterations and pos_err < self.pos_error_thresh:
                keep_executing = False
            self.loop_rate.sleep()

        # Observe outcome and produce new states
        new_states = copy.deepcopy(states)
        new_states.x = round(self.true_pos_x)
        new_states.y = round(self.true_pos_y)
        #new_states.has_food = self.food_sensor
        if move_action:
            new_states.battery = states.battery - 1
            if new_states.battery < 0:
                new_states.battery = 0
        if grab_action: 
            # Find which food index is the food at the current location, if there is one
            food_index = -1
            for i in range(num_food):
                if food_pos[i][0] == new_states.x and food_pos[i][1] == new_states.y:
                    food_index = i
                    break
            # Check if food was found at the current location
            if food_index > -1:
                if isinstance(states, SwarmStates):
                    new_states.food_cluster = food_cluster[food_index]
                food_map[new_states.x, new_states.y] = 0 # Remove food from robot's location on map
                new_states.food_state = getBinaryFromFoodMap(food_map, num_food, food_pos)
        if new_states.has_food == False and isinstance(states, SwarmStates):
            new_states.food_cluster = -1

        return new_states


    def poseCallback(self, msg):
        self.true_pos_x = msg.transform.translation.x
        self.true_pos_y = msg.transform.translation.y

    #def foodSensorCallback(self, msg):
    #    self.food_sensor = msg.data
