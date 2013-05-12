#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from threading import Lock

state_lock = Lock()
state = None

goal_lock = Lock()
goal = None

def state_callback(msg):
   rospy.loginfo(rospy.get_name() + " received new state: %s" % msg)
   with state_lock:
      global state
      state = msg

def goal_callback(msg):
   rospy.loginfo(rospy.get_name() + " received new goal: %s" % msg)
   with goal_lock:
      global goal
      goal = msg

def land_callback(msg):
   rospy.loginfo(rospy.get_name() + " UAV land requested - setting goal to none.")
   with goal_lock:
      global goal
      goal = None

# Tells the UAV to hold at its current position
def stop_uav(pub):
   pass
   
# Tells the UAV to move toward the current goal
def move_uav(pub,curState,curGoal):
   pass

class Namespace: pass

def simple_nav():
   pub = rospy.Publisher('control', String)
   rospy.init_node('simple_nav')
   rospy.Subscriber("state", String, state_callback)
   rospy.Subscriber("goal", String, goal_callback)
   rospy.Subscriber("land", String, land_callback)

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

if __name__ == '__main__':
   try:
      simple_nav()
   except rospy.ROSInterruptException:
      pass
