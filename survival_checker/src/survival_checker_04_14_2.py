#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import socket
import rosnode
import rostopic
import roslib.packages
import roslib.message
import roslib.names
import rosgraph
import topic_tools
import rosgraph.masterapi
from operator import itemgetter
from std_msgs.msg import String
from autoware_msgs.msg import NotSurvivaNodeAndTopic

def master_get_topic_types(master):
	try:
		val = master.getTopicTypes()
	except xmlrpclib.Fault:
		#TODO: remove, this is for 1.1
		sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
		val = master.getPublishedTopics('/')
	return val

def msgevalgen(pattern):
	"""
	Generates a function that returns the relevant field (aka 'subtopic') of a Message object
	@param pattern: subtopic, e.g. /x. Must have a leading '/' if specified.
	@type  pattern: str
	@return: function that converts a message into the desired value
	@rtype: fn(rospy.Message) -> value
	"""
	if not pattern or pattern == '/':
		return None
	def msgeval(msg):
		# I will probably replace this with some less beautiful but more efficient
		try:
			return eval('msg'+'.'.join(pattern.split('/')))
		except AttributeError as e:
			sys.stdout.write("no field named [%s]"%pattern+"\n")
			return None
	return msgeval



def get_topic_type(topic):
	"""
	subroutine for getting the topic type
	@return: topic type, real topic name and fn to evaluate the message instance
	if the topic points to a field within a topic, e.g. /rosout/msg
	@rtype: str, str, fn
	"""
	try:
		val = master_get_topic_types(rosgraph.masterapi.Master('/rostopic'))
	except socket.error:
		raise ROSTopicIOException("Unable to communicate with master!")

	# exact match first, followed by prefix match
	matches = [(t, t_type) for t, t_type in val if t == topic]
	if not matches:
		matches = [(t, t_type) for t, t_type in val if topic.startswith(t+'/')]
		# choose longest match
		matches.sort(key=itemgetter(0), reverse=True)
	if matches:
		t, t_type = matches[0]
		if t_type == roslib.names.ANYTYPE:
			return None, None, None
		return t_type, t, msgevalgen(topic[len(t):])
	else:
		return None, None, None

#ノードとトピックの監視クラス
class SurvivalChecker:
	__check_node_list_ = []  #監視ノード名の一覧
	__check_topic_list_ = [] #監視トピック名の一覧

	#監視ノード一覧を/params/node_check_listから取得
	def __getNodeList(self):
		node_list_path = roslib.packages.get_pkg_dir('survival_checker') + '/params/node_check_list'
		try:		
			ifs = open(node_list_path, 'r') 
		except IOError:
			print 'error: not open ' + node_list_path
			return

		self.__check_node_list_ = ifs.read().splitlines() #改行を省くライン読み込み
		for str in self.__check_node_list_:
			if str[0] == '#': #コメント行を省く
				self.__check_node_list_.remove(str)
			else:
				print "node:" + str
		ifs.close()
		#print 'read ' + node_list_path

	#監視トピック一覧を/params/topic_check_listから取得
	def __getTopicList(self):
		topic_list_path = roslib.packages.get_pkg_dir('survival_checker') + '/params/topic_check_list'
		try:		
			ifs = open(topic_list_path, 'r') 
		except IOError:
			print 'error: not open ' + topic_list_path
			return

		self.__check_topic_list_ = ifs.read().splitlines() #改行を省くライン読み込み
		for str in self.__check_topic_list_:
			if str[0] == '#': #コメント行を省く
				self.__check_topic_list_.remove(str)
			else:
				print "topic:" + str
		ifs.close()
		#print 'read ' + topic_list_path

	#ノードの生存確認
	def __nodeCheck(self):
		node_list = rosnode.get_node_names() #現在のノードプロセス一覧を取得
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

	#トピックの生存確認
	def __topicCheck(self):
		master = rosgraph.masterapi.Master('/rostopic') #rosgraphのマスター取得

		state = master.getSystemState()
		all_pubs, all_subs, _ = state

		for check_topic in self.__check_topic_list_:
			topic = rosgraph.names.script_resolve_name('rostopic', check_topic)
			subs = [x for x in all_subs if x[0] == topic]
			pubs = [x for x in all_pubs if x[0] == topic]
			#if len(pubs) != 0:
			#	print pubs

			#topic_type, real_topic, msg_eval = get_topic_type(topic)
			#print topic_type, real_topic, msg_eval
		return ''

	#ノードとトピックのチェックを一定周期で行う
	def __timerCallback(self, data):
		pubdata = NotSurvivaNodeAndTopic()
		pubdata.node = self.__nodeCheck()
		pubdata.topic = self.__topicCheck()

		pubdata.header.stamp = rospy.Time.now()
		self.__pub_not_survival_node_.publish(pubdata)

	def __init__(self):
		rospy.init_node('survival_checker')

		self.__getNodeList()
		self.__getTopicList()

		self.__pub_not_survival_node_ = rospy.Publisher('/not_survival_node_and_topic', NotSurvivaNodeAndTopic, queue_size=10)
		rospy.Timer(rospy.Duration(0.05), self.__timerCallback)

if __name__ == '__main__':
	print sys.argv
	checker = SurvivalChecker()
	rospy.spin()
