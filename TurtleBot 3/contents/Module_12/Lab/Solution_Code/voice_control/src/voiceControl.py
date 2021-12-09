#!/usr/bin/env python

"""
	VOICE CONTROL

	 Program to :
        - Subscribe to the topic "speechRecognizer/OutputString" and capture words/sentences recognized from "audioTreatment.py"
	- process the words, and compare them to different list of "key"words
	- depending on the word/sentence spoken, order the robot to move in certains directions
		- create a publisher to the topic "cmd_vel" which control the speed of the robot
		- send "Twist" type messages to update the speed and the direction of the robot

    This program don't need specific libraries, a part of std_msgs from the ROS environment (to receive String objects)
    This program is launched through the launch file in the ROS package

"""
# ROS imports
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

"""
	MAIN CLASS

	This class :
		- defines the keyword lists to be compared to the input words
		- print a welcome message to show how to use the program
		- Subscribe to the topic "speechRecognizer/OutputString" and capture words/sentences recognized from "audioTreatment.py"
		- send "Twist" type messages to the topic "cmd_vel" depending on the words/sentences spoken, to update the robot's movement speed and direction

"""
class voiceControl(object):

	#---------------------------------------------------------------------------------------
	#	Init all the class parameters :
	#	ROS node, publisher, subscriber, keyword lists, initial robot speed
	#---------------------------------------------------------------------------------------
	def __init__(self):

		# node initialisation
		rospy.loginfo("Creating ROS node")
		rospy.init_node('voiceControl', anonymous=True)
		rospy.loginfo("Successfully created 'voiceControl' node")
		rospy.on_shutdown(self.shutdown)

		# Print introduction (menu) message
		self.initMsg()

		# keyord list : you can add more words in each list if you want, depending on the action
		self.keyword_left = ["left", "turn left", "go left", "moove left", "rotate left", "let", "lef"]
		self.keyword_right = ["right", "turn right", "go right", "moove right", "rotate right", "righ"]
		self.keyword_forward = ["forward", "go forward", "moove forward", "front", "go front", "moove front", "straight", "go straight", "moove straight"]
		self.keyword_back = ["back", "backward", "go back", "go backward", "moove back", "moove backward", "rear", "go rear", "moove rear"]
		self.keyword_stop = ["stop", "stop yourself", "don't moove", "dont moove"]
		self.keyword_accelerate = ["accelerate", "faster", "moove faster", "go faster"]
		self.keyword_brake = ["brake", "decelerate" "slower", "moove slower", "go slower"]
		self.keyword_hello = ["hello", "hi"]

		self.cmd_vel = Twist()

		# Robot stopped at start (3 first digits : linear x,y,z movement)(3 last digits : angular x,y,z movement)
		rospy.loginfo("setting the robot speed to 0 (stop it)")
		self.vel_robot = self.configRobotSpeed(0,0,0,0,0,0)
		rospy.loginfo("Successfully set robot initial speed to 0")

		# Subscriber and Publisher configuration
		rospy.loginfo("Initializing robot speed command publisher")
		self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
		rospy.loginfo("Successfully initialized publisher 'cmd_vel'")

		rospy.loginfo("Initializing String recognized subscriber")
		rospy.Subscriber("speechRecognizer/outputString", String, self.callbackAudio)
		rospy.loginfo("Successfully subscribed to the topic 'speechRecognizer/outputString'")

		# continue to execute the node until it is stopped
		rospy.spin()
		pass

	#---------------------------------------------------------------------------------------
	#	Welcome message to show how the program works
	#---------------------------------------------------------------------------------------
	def initMsg(self):
		print("\n\t\t###################################################################")
		print("\t\t-                                                                 -")
		print("\t\t-                 Turtlebot3 Voice control                        -")
		print("\t\t-                                                                 -")
		print("\t\t###################################################################")

		print("\n\t~~~ This programs allow you to control the Turtlebot3 robot from voice")
		print("\n\t~~~ Ensure you have launched the program using 'roslaunch' and that the audioTreatment program is running")

		print("\n\tAvailable commands : 'turn left', 'turn right', 'forward', 'backward', 'accelerate', 'decelerate' 'hello' and 'stop'")
		print("\n\tYou can also say rational commands as 'front', 'back', 'left', 'right', 'faster,' slower ...")
		pass

	#---------------------------------------------------------------------------------------
	#	Function that configure a twist message to control robot speed and direction
	#	modify linear speed (x, y, z) and angular speed (x, y, z)
	#---------------------------------------------------------------------------------------
	def configRobotSpeed(self, linx, liny, linz, angx, angy, angz):

		twistmessage = Twist()
		twistmessage.linear.x = linx
		twistmessage.linear.y = liny
		twistmessage.linear.z = linz
		twistmessage.angular.x = angx
		twistmessage.angular.y = angy
		twistmessage.angular.z = angz
		return twistmessage
		pass

	#---------------------------------------------------------------------------------------
	#	Function that configure update a twist message to change the robot's speed and direction
	#	modify linear speed (x, y, z) and angular speed (x, y, z)
	#---------------------------------------------------------------------------------------
	def updateRobotSpeed(self, actualspeed, linx = 0, liny = 0, linz = 0, angx = 0, angy = 0, angz = 0):

		twistmessage = actualspeed
		twistmessage.linear.x += linx
		twistmessage.linear.y += liny
		twistmessage.linear.z += linz
		twistmessage.angular.x += angx
		twistmessage.angular.y += angy
		twistmessage.angular.z += angz
		return twistmessage
		pass

	#---------------------------------------------------------------------------------------
	#	Function to accelerate depending on an acceleration constant and the actual speed of the robot
	#	returns a TwistMessage
	#---------------------------------------------------------------------------------------
	def decideRobotAccel(self, actualspeed):
		twistmessage = actualspeed
		accelerate_constant = 0.1
		event = 0
		if (twistmessage.linear.x) > 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, accelerate_constant, 0, 0, 0, 0, 0)
		elif (twistmessage.linear.x) < 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, -accelerate_constant, 0, 0, 0, 0, 0)

		if (twistmessage.linear.y) > 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, accelerate_constant, 0, 0, 0, 0)
		elif (twistmessage.linear.y) < 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, -accelerate_constant, 0, 0, 0, 0)

		if (twistmessage.linear.z) > 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, accelerate_constant, 0, 0, 0)
		elif (twistmessage.linear.z) < 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, -accelerate_constant, 0, 0, 0)

		if (twistmessage.angular.x) > 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, 0, accelerate_constant, 0, 0)
		elif (twistmessage.angular.x) < 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, 0, -accelerate_constant, 0, 0)

		if (twistmessage.angular.y) > 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, 0, 0, accelerate_constant, 0)
		elif (twistmessage.angular.y) < 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, 0, 0, -accelerate_constant, 0)

		if (twistmessage.angular.z) > 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, 0, 0, 0, accelerate_constant)
		elif (twistmessage.angular.z) < 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, 0, 0, 0, -accelerate_constant)

		if (event == 0):
			print("\tthe robot is stopped ! It can't accelerate")

		return twistmessage
		pass

	#---------------------------------------------------------------------------------------
	#	Function to deccelerate depending on an decceleration constant and the actual speed of the robot
	#	returns a TwistMessage
	#---------------------------------------------------------------------------------------
	def decideRobotDecel(self, actualspeed):

		twistmessage = actualspeed
		decelerate_constant = 0.1
		event = 0
		if (twistmessage.linear.x) > 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, -decelerate_constant, 0, 0, 0, 0, 0)
		elif (twistmessage.linear.x) < 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, decelerate_constant, 0, 0, 0, 0, 0)

		if (twistmessage.linear.y) > 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, -decelerate_constant, 0, 0, 0, 0)
		elif (twistmessage.linear.y) < 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, decelerate_constant, 0, 0, 0, 0)

		if (twistmessage.linear.z) > 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, -decelerate_constant, 0, 0, 0)
		elif (twistmessage.linear.z) < 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, decelerate_constant, 0, 0, 0)

		if (twistmessage.angular.x) > 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, 0, -decelerate_constant, 0, 0)
		elif (twistmessage.angular.x) < 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, 0, decelerate_constant, 0, 0)

		if (twistmessage.angular.y) > 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, 0, 0, -decelerate_constant, 0)
		elif (twistmessage.angular.y) < 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, 0, 0, decelerate_constant, 0)

		if (twistmessage.angular.z) > 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, 0, 0, 0, -decelerate_constant)
		elif (twistmessage.angular.z) < 0:
			event = 1
			twistmessage = self.updateRobotSpeed(twistmessage, 0, 0, 0, 0, 0, decelerate_constant)

		if (event == 0):
			print("\tthe robot is stopped ! It can't decelerate")

		return twistmessage
		pass

	#---------------------------------------------------------------------------------------
	#	Function that modify speed and orientation depending on the message received
	#---------------------------------------------------------------------------------------
	def callbackAudio(self, text):

		text.data = text.data.lower()   # put data in lowercase
		rospy.loginfo("data received : " + text.data)
		if ((text.data in self.keyword_left) or ("left" in text.data)):
			rospy.loginfo("received 'turn left' call")
			self.cmd_vel = self.configRobotSpeed(0, 0, 0, 0, 0, 0.5)
			self.vel_publisher.publish(self.cmd_vel)
		elif ((text.data in self.keyword_right) or ("right" in text.data)):
			rospy.loginfo("received 'turn right' call")
			self.cmd_vel = self.configRobotSpeed(0, 0, 0, 0, 0, -0.5)
			self.vel_publisher.publish(self.cmd_vel)
		elif ((text.data in self.keyword_forward) or ("front" in text.data) or ("forward" in text.data) or ("straight" in text.data)):
			rospy.loginfo("received 'forward' call")
			self.cmd_vel = self.configRobotSpeed(-0.2, 0, 0, 0, 0, 0)
			self.vel_publisher.publish(self.cmd_vel)

		elif ((text.data in self.keyword_back) or ("back" in text.data) or ("backward" in text.data) or ("rear" in text.data)):
			rospy.loginfo("received 'backward' call")
			self.cmd_vel = self.configRobotSpeed(0.2, 0, 0, 0, 0, 0)
			self.vel_publisher.publish(self.cmd_vel)

		elif ((text.data in self.keyword_stop) or ("stop" in text.data)):
			rospy.loginfo("received 'stop' call")
			self.cmd_vel = self.configRobotSpeed(0, 0, 0, 0, 0, 0)
			self.vel_publisher.publish(self.cmd_vel)

		elif ((text.data in self.keyword_accelerate) or ("accelerate" in text.data) or ("faster" in text.data)):
			rospy.loginfo("received 'accelerate' call")
			self.cmd_vel = self.decideRobotAccel(self.cmd_vel)
			self.vel_publisher.publish(self.cmd_vel)

		elif ((text.data in self.keyword_brake) or ("brake" in text.data) or ("decelerate" in text.data) or ("slower" in text.data)):
			rospy.loginfo("received 'decelerate' call")
			self.cmd_vel = self.decideRobotDecel(self.cmd_vel)
			self.vel_publisher.publish(self.cmd_vel)

		elif ((text.data in self.keyword_hello) or ("hello" in text.data) or ("hi" in text.data)):
			rospy.loginfo("Hi ! My name is Turtlebot 3")

		pass

	#-----------------------------------------------------------------
	# in case of forced stop, as CTRL + C, print an information message
	#-----------------------------------------------------------------
	def shutdown(self):
		rospy.logwarn("Closing voice control program")
		pass

#-----------------------------------------------------------------
#  main function, used to start the program
#-----------------------------------------------------------------
if __name__ == "__main__":
	try:
		# start the program (run the init function from the main class)
		start = voiceControl()
	except:
		rospy.logerr("an error (exception) as occured, closing program")
	pass
