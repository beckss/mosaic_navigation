#!/usr/bin/env python
"""
ROS node for navigating quadrotor like UAVs toward a specified goal.

This is primarily designed to work with the ardrone_autonomy package for
controlling parrot ardrones, but it may have applications beyond this.

ROS Parameters:
NAME
publish_frequency - When goal is set, specifies how often (in Hertz) control
                    messages should be sent on the control topic.

min_speed   - Minimum speed for UAV to travel at (default: 0)
max_speed   - Maximum speed for UAV to travel at (default: 1)
speed_scale - Multiplied by current distance to goal, to determine the target
              speed of travel. If the result is not in [min_speed,max_speed]
              then it is reset to either min_speed or max_speed as appropriate.

Subscriptions:
TOPIC   TYPE                PURPOSE
/state  nav_msgs.Odometry   Specifies the UAVs current pose and trajectory in
                            its local cartesian coordinate frame
                            (i.e. x & y in metres not lat/lon).

/goal   geometry_msgs.Pose  Sets the goal pose for the UAV. The position
                            is specified in the same local cartesian coordinate
                            frame as the state Odometry messages.

/reset  nav_msgs.Empty      An empty message set to this topic, resets the goal
                            state to none, and stops the navigator sending
                            commands to the UAV until a new goal is set. This
                            topic can be shared with ardrone_autonomy package
                            so that the ardrone receives the reset command
                            regardless of whether the navigator is running or
                            not.

/land   std_msgs.Empty      Same effect as the reset topic. This is designed
                            to be shared with the ardrone_autonomy land topic,
                            so that the navigator stops sending velocity
                            commands once the UAV has been requested to land.

Publications:
TOPIC      TYPE                   PURPOSE
/control   geometry_msgs.Twist    Sends velocity commands to the UAV to change
                                  its location in the local cartesian
                                  coordinate frame. Designed to be remapped
                                  to the ardrone_autonomy /cmd_vel topic.
                                  Commands are only sent on this topic if that
                                  current goal pose is not None.
"""
import rospy
import numpy as np
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from threading import Lock

class Namespace:
   """
   Empty class used for managing variable scope.
   
   This class serves no other purpose and so can be safely ignored.
   """
   pass

# Current UAV state extracted from Odometry
# Lock used to ensure thread safety
state_lock = Lock()
state = None

# Current Goal position - if any
# Lock used to ensure thread safety
goal_lock = Lock()
goal = None

# speed parameters
minSpeed = 0
maxSpeed = 1.0
speedScale = 0.5

def state_callback(msg):
   """
   ROS Callback used to update navigator node about the UAV's current state.

   Usage: state_callback(msg) where message of type nav_msgs.Odometry
   We assume that the position is specified in the UAV's local cartesian
   coordinate frame - i.e. in metres rather than lat/lon.
   """
   rospy.logdebug(rospy.get_name() + " received new state: %s" % msg)
   with state_lock:
      global state
      pos = msg.pose.pose.position
      att = msg.pose.pose.orientation
      state = Namespace()
      state.orientation = np.array([att.x, att.y, att.z, att.w])
      state.position = np.array([pos.x, pos.y, pos.z])

def goal_callback(msg):
   """
   ROS Callback used to update the navigator's current goal position.

   Usage: goal_callback(msg) where message of type geometry_msgs.Pose
   We assume that the position is specified in the UAV's local cartesian
   coordinate frame - i.e. in metres rather than lat/lon.
   """
   rospy.loginfo(rospy.get_name() + " received new goal: %s" % msg)
   with goal_lock:
      global goal
      pos = msg.position
      att = msg.orientation
      goal = Namespace()
      goal.orientation = np.array([att.x, att.y, att.z, att.w])
      goal.position= np.array([pos.x, pos.y, pos.z])

def reset_callback(msg):
   """
   ROS Callback for reset topic.

   If an empty message is set to this topic,
   then the navigator's goal is set to none, so that it stops sending commands
   to the UAV. For ardrones, this topic may be shared with ardrone_autonomy, so
   that the UAV also receives the reset request, regardless of whether the 
   navigator is operating correctly or not.
   """
   rospy.loginfo(rospy.get_name() + " UAV reset requested - deleting goal.")
   with goal_lock:
      global goal
      goal = None

def land_callback(msg):
   """
   ROS Callback for land topic.

   If an empty message is set to this topic,
   then the navigator's goal is set to none, so that it stops sending commands
   to the UAV. For ardrones, this topic may be shared with ardrone_autonomy, so
   that the UAV also receives the land request directly, regardless of whether
   the navigator is operating correctly or not.
   """
   rospy.loginfo(rospy.get_name() + " UAV land requested - deleting goal.")
   with goal_lock:
      global goal
      goal = None

def stop_uav(pub):
   """
   Tells the UAV to hold at its current position.
   """
   zero = Vector3(0,0,0)
   msg = Twist()
   msg.linear = zero
   msg.angular = zero
   pub.publish(msg)
   
def move_uav(pub,curState,curGoal):
   """
   Tells the UAV to move toward the current goal pose.

   Orientation is currently ignored, but at some point we will want to use this
   to control UAV yaw as well.
   """

   # Calculate the direction we want to move in
   diff = curGoal.position-curState.position
   dist = np.linalg.norm(diff)
   direction = diff/dist

   # Calculate the speed we want to move at, and use this to set
   # flight vector (direction*speed)
   speed = min(max(dist*speedScale,minSpeed),maxSpeed)
   flightVec = direction*speed

   str = "speed parameters: (min: %f max: %f scale: %f" % \
      (minSpeed,maxSpeed,speedScale)

   rospy.logdebug(str)
   rospy.logdebug("position: %s" % curState.position)
   rospy.logdebug("target: %s" % curGoal.position)
   rospy.logdebug("dist: %s" % dist)
   rospy.logdebug("speed: %s" % speed)
   rospy.logdebug("flightVec: %s" % flightVec)

   # publish Twist message to tell UAV to move according to the
   # flight vector
   msg = Twist()
   msg.angular = Vector3(0,0,0)
   msg.linear = Vector3()
   msg.linear.x = flightVec[0]
   msg.linear.y = flightVec[1]
   msg.linear.z = flightVec[2]
   pub.publish(msg)

def simple_nav():
   """
   Main function for simple navigator.

   This function creates a ROS node used to move quadrotor like UAVs in a linear
   trajectory towards some goal state. See module documentation for details.
   """
   pub = rospy.Publisher('control', Twist)
   rospy.init_node('simple_nav')
   rospy.Subscriber("state", Odometry, state_callback)
   rospy.Subscriber("goal", Pose, goal_callback)
   rospy.Subscriber("land", Empty, land_callback)
   rospy.Subscriber("reset", Empty, reset_callback)

   global minSpeed
   global maxSpeed
   global speedScale

   pns = rospy.get_name()
   minSpeed = rospy.get_param(pns + '/min_speed', 0)
   maxSpeed = rospy.get_param(pns + '/max_speed', 1)
   speedScale = rospy.get_param(pns + '/speed_scale', 0.5)
   publishFrequency = rospy.get_param(pns + '/publish_frequency',1.0)

   ns = Namespace()  
   ns.goal = None
   ns.state = None

   while not rospy.is_shutdown():
   
      with goal_lock:
         ns.goal = goal

      with state_lock:
         ns.state = state

      if ns.goal is None:
         rospy.loginfo("No goal set.")

      elif ns.state is None:
         rospy.logwarn("Goal is set, but UAV state is unknown!")
         rospy.loginfo("Halting UAV.")
         stop_uav(pub)

      else:
         rospy.loginfo("Moving UAV toward goal.")
         move_uav(pub,ns.state,ns.goal)

      rospy.sleep(1/publishFrequency)

   rospy.loginfo("Controlling is shutting down.")
   rospy.loginfo("Stopping UAV.")
   stop_uav(pub)

if __name__ == '__main__':
   try:
      simple_nav()
   except rospy.ROSInterruptException:
      pass
