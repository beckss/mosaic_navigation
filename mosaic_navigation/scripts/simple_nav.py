#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from threading import Lock

# Empty class used for managing variable scope
class Namespace: pass

state_lock = Lock()
state = None

goal_lock = Lock()
goal = None

def state_callback(msg):
   rospy.loginfo(rospy.get_name() + " received new state: %s" % msg)
   with state_lock:
      global state
      pos = msg.pose.pose.position
      att = msg.pose.pose.orientation
      state = Namespace()
      state.orientation = np.array([att.x, att.y, att.z, att.w])
      state.position = np.array([pos.x, pos.y, pos.z])

def goal_callback(msg):
   rospy.loginfo(rospy.get_name() + " received new goal: %s" % msg)
   with goal_lock:
      global goal
      pos = msg.position
      att = msg.orientation
      goal = Namespace()
      goal.orientation = np.array([att.x, att.y, att.z, att.w])
      goal.position= np.array([pos.x, pos.y, pos.z])

def reset_callback(msg):
   rospy.loginfo(rospy.get_name() + " UAV reset requested - setting goal to none.")
   with goal_lock:
      global goal
      goal = None

def land_callback(msg):
   rospy.loginfo(rospy.get_name() + " UAV land requested - setting goal to none.")
   with goal_lock:
      global goal
      goal = None

# Tells the UAV to hold at its current position
def stop_uav(pub):
   zero = Vector3(0,0,0)
   msg = Twist()
   msg.linear = zero
   msg.angular = zero
   pub.publish(msg)
   
# Tells the UAV to move toward the current goal
# orientation is currently ignored, but at some point we will want to set yaw
def move_uav(pub,curState,curGoal):
   diff = curGoal.position-curState.position
   dist = np.linalg.norm(diff)
   direction = diff/dist

   targetSpeed = 2
   maxSpeed = 1
   minSpeed = 0

   speed = min(max(dist/targetSpeed,minSpeed),maxSpeed)
   acceleration = direction*speed

   rospy.logwarn("position: %s" % curState.position)
   rospy.logwarn("target: %s" % curGoal.position)
   rospy.logwarn("dist: %s" % dist)
   rospy.logwarn("speed: %s" % speed)
   rospy.logwarn("acceleration: %s" % acceleration)

   msg = Twist()
   msg.angular = Vector3(0,0,0)
   msg.linear = Vector3()
   msg.linear.x = acceleration[0]
   msg.linear.y = acceleration[1]
   msg.linear.z = acceleration[2]
   pub.publish(msg)

def simple_nav():
   pub = rospy.Publisher('control', Twist)
   rospy.init_node('simple_nav')
   rospy.Subscriber("state", Odometry, state_callback)
   rospy.Subscriber("goal", Pose, goal_callback)
   rospy.Subscriber("land", Empty, land_callback)
   rospy.Subscriber("reset", Empty, reset_callback)

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

      rospy.sleep(1.0)

   rospy.loginfo("Controlling is shutting down.")
   rospy.loginfo("Stopping UAV.")
   stop_uav(pub)

if __name__ == '__main__':
   try:
      simple_nav()
   except rospy.ROSInterruptException:
      pass
