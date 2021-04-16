#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import rosnode
import rostopic
import roslib.packages
import rosgraph
import topic_tools
from std_msgs.msg import String
from autoware_msgs.msg import NotSurvivaNodeAndTopic

#監視するトピック情報クラス
class TopicInfo:
	__DEFAULT_CHECK_TIME_ = 1.0 #デフォルトチェックタイム
	__sub_time_ = rospy.Time(0) #トピックをsubscribeした時間

	#コンストラクタの引数は,トピック名,トピックの型,超過時間しきい値
	def __init__(self, topic_name, topic_type, check_time):
		self.__name_ = topic_name
		self.__sub_ = rospy.Subscriber(topic_name, topic_type, self.__callback)
		if check_time > 0:		
			self.__check_time_ = check_time
		else:
			self.__check_time_ = __DEFAULT_CHECK_TIME_

	def __callback(self, msg):
		self.__sub_time_ = rospy.Time.now()

	#現在の時間 - subscribeした時間 > 超過時間しきい値ならFalse,違うならTrue
	def getDiffTime(self):
		topic = rosgraph.names.script_resolve_name('rostopic', self.__name_)
		master = rosgraph.masterapi.Master('/rostopic')
		state = master.getSystemState()
		pubs, subs, _ = state
		subs = [x for x in subs if x[0] == topic]
		pubs = [x for x in pubs if x[0] == topic]
		print pubs

		nowtime = rospy.Time.now()
		ros_dt = nowtime - self.__sub_time_
		dt = ros_dt.to_sec() + ros_dt.to_nsec() * 1E-9
		if dt > self.__check_time_:
			return False
		else:
			return True

	#トピック名を返す
	def getTopicName(self):
		return self.__name_

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
		for list in self.__check_node_list_:
			if list[0] == '#': #コメント行を省く
				self.__check_node_list_.remove(list)
			else:
				print "node:" + list
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

		list = ifs.read().splitlines() #改行を省くライン読み込み
		for str in list:
			if str[0] == '#': #コメント行を省く
				continue

			params = str.split(',')
			info = TopicInfo(params[0], globals()[params[1]], float(params[2]))
			self.__check_topic_list_.append(info)
			print "topic:" + params[0]
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
		ret = []
		for check_topic in self.__check_topic_list_:
			if check_topic.getDiffTime() == False:
				ret.append(check_topic.getTopicName())
				#print "OK"
		return ret

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
