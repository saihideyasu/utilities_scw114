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
import itertools
from operator import itemgetter
from std_msgs.msg import String
from autoware_msgs.msg import SurvivalReport

#rostopic.pyから抜粋
def master_get_topic_types(master):
	try:
		val = master.getTopicTypes()
	except xmlrpclib.Fault:
		#TODO: remove, this is for 1.1
		sys.stderr.write("WARNING: rostopic is being used against an older version of ROS/roscore\n")
		val = master.getPublishedTopics('/')
	return val

#rostopic.pyから抜粋
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

#rostopic.pyから抜粋
def get_topic_type(topic, blocking=False):
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

#rostopic.pyから抜粋
def get_topic_class(topic, blocking=False):
    """
    Get the topic message class
    @return: message class for topic, real topic
    name, and function for evaluating message objects into the subtopic
    (or None)
    @rtype: roslib.message.Message, str, str
    @raise ROSTopicException: if topic type cannot be determined or loaded
    """
    topic_type, real_topic, msg_eval = get_topic_type(topic, blocking=blocking)
    if topic_type is None:
        return None, None, None
    msg_class = roslib.message.get_message_class(topic_type)
    if not msg_class:
        raise ROSTopicException("Cannot load message class for [%s]. Are your messages built?"%topic_type)
    return msg_class, real_topic, msg_eval

#トピックcallbackクラス
class TopicCallback:
	__sub_time = rospy.Time(0)

	def __init__(self, real_topic, normal_topic, msg_class, topic):
		self.__sub = rospy.Subscriber(real_topic, msg_class, self.__callback, topic)
		self.__name = normal_topic

	def __callback(self, data, topic, current_time=None):
		self.__sub_time = rospy.Time.now()

	def getName(self):
		return self.__name

	def getTimeDt(self, nowtime):
		rostime_dt = nowtime - self.__sub_time
		dt = rostime_dt.secs + rostime_dt.nsecs * 1E-9
		return dt

def findTopicCallback(topic_callback, find_name):
	for callback in topic_callback:
		if callback.getName() == find_name:
			return callback
	return None

#ノードとトピックの監視クラス
class SurvivalChecker:
	__check_node_list_ = []  #監視ノード名の一覧
	__check_topic_list_ = [] #監視トピック名の一覧
	__topic_callbacks = [] #subscribeしたトピック一覧

	#監視ノード一覧を/params/node_check_listから取得
	def __getNodeList(self):
		node_list_path = roslib.packages.get_pkg_dir('survival_checker') + '/params/node_check_list'
		try:		
			ifs = open(node_list_path, 'r') 
		except IOError:
			sys.stderr.write("error: not open node list\n")
			return

		self.__check_node_list_ = ifs.read().splitlines() #改行を省くライン読み込み
		for str in self.__check_node_list_:
			if str[0] == '#': #コメント行を省く
				self.__check_node_list_.remove(str)
			else:
				print "node:" + str
		ifs.close()

	#監視トピック一覧を/params/topic_check_listから取得
	def __getTopicList(self):
		topic_list_path = roslib.packages.get_pkg_dir('survival_checker') + '/params/topic_check_list'
		try:		
			ifs = open(topic_list_path, 'r') 
		except IOError:
			sys.stderr.write("error: not open topic list\n")
			return

		topiclist = ifs.read().splitlines() #改行を省くライン読み込み
		for line in topiclist:
			if line[0] == '#': #コメント行を省く
				continue
			else:
				appdata = str.split(line, ',')
				print appdata
				self.__check_topic_list_.append(appdata)
		ifs.close()

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
		return ret

	#トピックの生存確認
	def __topicCheck(self, nowtime):
		ret_no_survival_topic = []  #subscribeされていないトピック一覧
		ret_not_relevant_topic = [] #登録情報と同じpublishノードが存在しないトピック一覧
		ret_no_response_topic = []  #一定時間subscribeされていないノード一覧

		master = rosgraph.masterapi.Master('/rostopic') #rosgraphのマスター取得

		state = master.getSystemState()
		all_pubs, all_subs, _ = state

		for check_topic_info in self.__check_topic_list_:
			#check_topic_name:チェックするトピック名
			#check_pub_node:  チェックするトピックをpublishする予定のノード
			#check_dt:        チェックするトピックの生存時間判定
			check_topic_name, check_pub_node, check_dt = check_topic_info
			check_dt = float(check_dt)

			#すでに登録されているトピックからcheck_topic_nameと同じトピックを検索
			#ない場合はret_no_survival_topicにトピック名を追加
			topic_callback = findTopicCallback(self.__topic_callbacks, check_topic_name)
			if(topic_callback is None):
				ret_no_survival_topic.append(check_topic_name)
				continue

			#check_topic_nameのpublishノードとsubscribeノードを検索
			try:
				topic = rosgraph.names.script_resolve_name('rostopic', check_topic_name)
				#subs = [x for x in all_subs if x[0] == topic]
				pubs = [x for x in all_pubs if x[0] == topic]
			except socket.error:
				sys.stderr.write("error: topic info2\n")
				continue

			#check_pub_nodeと同じpublishノードが存在しない場合(該当ノードがpublishしていない)、ret_not_relevant_topicにトピック名を追加
			pub_node_check = False
			if pubs:
				for p in itertools.chain(*[l for x, l in pubs]):
					if check_pub_node == p:
						pub_node_check = True
						break
			if pub_node_check == False:
				ret_not_relevant_topic.append(check_topic_name)
				continue

			#トピックから応答が一定時間ない場合、ret_no_response_topicにトピック名を追加
			if topic_callback.getTimeDt(nowtime) > check_dt:
				ret_no_response_topic.append(check_topic_name)

		print 'survival'
		print ret_no_survival_topic
		print 'relevant'
		print ret_not_relevant_topic
		print 'response'
		print ret_no_response_topic
		return [ret_no_survival_topic, ret_not_relevant_topic, ret_no_response_topic]

	#現在のpublishさてているトピックを検索して、チェックトピック一覧に存在するトピックをsubscribeする
	def __subscriberSearch(self):
		for check_topic_info in self.__check_topic_list_:
			#check_topic_name:チェックするトピック名
			#check_pub_node:  チェックするトピックをpublishする予定のノード
			#check_dt:        チェックするトピックの生存時間判定
			check_topic_name, check_pub_node, check_dt = check_topic_info

			#チェックするトピックの現在情報を取得
			#topic_type:トピックの型  real_topic:リアルトピック(表示すると通常のトピック名と同じだった)
			#msg_eval:不明(std_msgs/StringではNone)
			try:
				rostopic_data = rosgraph.names.script_resolve_name('rostopic', check_topic_name)
				topic_type, real_topic, msg_eval = get_topic_class(rostopic_data, blocking=True)
			except socket.error:
				sys.stderr.write("error: topic info1\n")
				continue

			#トピックの型か無い、またはトピック名が無い場合は、すでにsubscribeしているものを破棄
			if(topic_type is None or real_topic is None):
				for topic_callback in self.__topic_callbacks:
					if topic_callback.getName() == check_topic_name:
						self.__topic_callbacks.remove(topic_callback)
						break
				continue

			#すでにsubscribeしているトピックに同名トピックが無いかを確認
			sub_flag = False
			for topic_callback in self.__topic_callbacks:
				if topic_callback.getName() == check_topic_name:
					sub_flag = True
					break

			#同名トピックが無い場合は、self.__topic_callbacksにsubscribe情報を追加
			if sub_flag == False:
				callback = TopicCallback(real_topic, check_topic_name, topic_type, rostopic_data)
				self.__topic_callbacks.append(callback)

	#ノードとトピックの生存確認
	def __survival_check(self):
		nowtime = rospy.Time.now()
		pubdata = SurvivalReport()
		pubdata.no_node = self.__nodeCheck()
		pubdata.no_survival_topic, pubdata.no_relevant_topic, pubdata.no_response_topic = self.__topicCheck(nowtime)

		pubdata.header.stamp = nowtime
		self.__pub_not_survival_node_.publish(pubdata)

	#ノードとトピックのチェックを一定周期で行う
	def __timerCallback(self, data):
		self.__subscriberSearch()
		self.__survival_check()

	def __init__(self):
		rospy.init_node('survival_checker')

		self.__getNodeList()
		self.__getTopicList()

		self.__pub_not_survival_node_ = rospy.Publisher('/survival_report', SurvivalReport, queue_size=10)
		rospy.Timer(rospy.Duration(0.05), self.__timerCallback)

if __name__ == '__main__':
	checker = SurvivalChecker()
	rospy.spin()
