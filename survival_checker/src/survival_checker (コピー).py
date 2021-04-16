#!/usr/bin/env python

import rospy
import rosnode
import rostopic
import roslib.packages
from std_msgs.msg import String
from autoware_msgs.msg import NotSurvivaNodeAndTopic

class TopicInfo:
	__sub_time_ = rospy.Time(0)
	__check_time = 1.0

	def __init__(self, topic_name, topic_type, check_time):
		self.__name_ = topic_name
		self.__sub_ = rospy.Subscriber(topic_name, topic_type, self.__callback)
		self.__check_time_ = check_time

	def __callback(self, msg):
		#print rospy.Time.now()
		self.__sub_time_ = rospy.Time.now()

	def getDiffTime(self):
		nowtime = rospy.Time.now()
		ros_dt = nowtime - self.__sub_time_
		dt = ros_dt.to_sec() + ros_dt.to_nsec() * 1E-9
		if dt > self.__check_time_:
			return False
		else:
			return True

	def getTopicName(self):
		return self.__name_

class NodeSurvivalChecker:
	__check_node_list_ = []
	__check_topic_list_ = []

	def __getNodeList(self):
		node_list_path = roslib.packages.get_pkg_dir('node_survival_checker') + '/params/node_check_list'
		try:		
			ifs = open(node_list_path, 'r') 
		except IOError:
			print 'error: not open ' + node_list_path
			return

		self.__check_node_list_ = ifs.read().splitlines()
		for list in self.__check_node_list_:
			if list[0] == '#':
				self.__check_node_list_.remove(list)
			else:
				print list

		ifs.close()
		print 'read ' + node_list_path

	def __getTopicList(self):
		topic_list_path = roslib.packages.get_pkg_dir('node_survival_checker') + '/params/topic_check_list'
		try:		
			ifs = open(topic_list_path, 'r') 
		except IOError:
			print 'error: not open ' + topic_list_path
			return

		list = ifs.read().splitlines()
		for str in list:
			params = str.split(',')
			info = TopicInfo(params[0], globals()[params[1]], float(params[2]))
			self.__check_topic_list_.append(info)
		print 'read ' + topic_list_path

	def __nodeCheck(self):
		node_list = rosnode.get_node_names()
		ret = []

		for check_node in self.__check_node_list_:
			check_flag = False
			for node_name in node_list:
				if node_name == check_node:
					check_flag = True
					break
			if check_flag == False:
				ret.append(check_node)
		#print ret
		return ret

	def __topicCheck(self):
		ret = []
		for check_topic in self.__check_topic_list_:
			if check_topic.getDiffTime() == False:
				ret.append(check_topic.getTopicName())
				#print "OK"
		return ret

	def __timerCallback(self, data):
		pubdata = NotSurvivaNodeAndTopic()
		pubdata.node = self.__nodeCheck()
		pubdata.topic = self.__topicCheck()

		pubdata.header.stamp = rospy.Time.now()
		self.__pub_not_survival_node_.publish(pubdata)

	def __init__(self):
		rospy.init_node('node_survival_checker')

		self.__getNodeList()
		self.__getTopicList()

		self.__pub_not_survival_node_ = rospy.Publisher('/not_survival_node_and_topic', NotSurvivaNodeAndTopic, queue_size=10)
		rospy.Timer(rospy.Duration(1), self.__timerCallback)

if __name__ == '__main__':
	checker = NodeSurvivalChecker()
	rospy.spin()
