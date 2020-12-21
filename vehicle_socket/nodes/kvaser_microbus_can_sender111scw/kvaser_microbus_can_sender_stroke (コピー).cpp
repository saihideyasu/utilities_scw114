#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <queue>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <can_msgs/Frame.h>
#include <mobileye_560_660_msgs/AftermarketLane.h>
#include <mobileye_560_660_msgs/ObstacleData.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_can_msgs/MicroBusCan501.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <autoware_can_msgs/MicroBusCanSenderStatus.h>
#include <autoware_can_msgs/MicroBusCanVelocityParam.h>
//#include <autoware_can_msgs/MicroBusCanSenderPositionCheck.h>
#include <autoware_config_msgs/ConfigMicroBusCan.h>
#include <autoware_config_msgs/ConfigVelocitySet.h>
#include <autoware_config_msgs/ConfigLocalizerSwitchFusion.h>
#include <autoware_config_msgs/ConfigCurrentVelocityConversion.h>
//#include <autoware_config_msgs/ConfigMicrobusInterface.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_msgs/PositionChecker.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_msgs/DetectedObjectArray.h>
#include <autoware_msgs/GnssStandardDeviation.h>
#include <autoware_msgs/DifferenceToWaypointDistance.h>
#include <autoware_msgs/NDTStat.h>
#include <autoware_msgs/LocalizerMatchStat.h>
#include <autoware_msgs/VehicleStatus.h>
#include <tf/tf.h>
#include "kvaser_can.h"
#include <time.h>

static const int SYNC_FRAMES = 50;
typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::TwistStamped, geometry_msgs::PoseStamped>
    TwistPoseSync;

class PID_params
{
private:
	double accel_e_prev_velocity_, brake_e_prev_velocity_;
	double accel_e_prev_acceleration_, brake_e_prev_acceleration_;
	double accel_e_prev_distance_, brake_e_prev_distance_;
	double steer_e_prev_distance_;

	double accel_diff_sum_velocity_, brake_diff_sum_velocity_;
	double accel_diff_sum_acceleration_, brake_diff_sum_acceleration_;
	double accel_diff_sum_distance_, brake_diff_sum_distance_;
	double steer_diff_sum_distance_;

	double stroke_prev_, stop_stroke_prev_;
	int stroke_state_mode_;
public:
	const static int STROKE_STATE_MODE_ACCEL_ = 0;
	const static int STROKE_STATE_MODE_BRAKE_ = 1;
	const static int STROKE_STATE_MODE_STOP_ = 2;
	const static int STROKE_STATE_MODE_KEEP_ = 3;

	PID_params() {}
	int init(double acc_stroke)
	{
		accel_e_prev_velocity_ = brake_e_prev_velocity_ = accel_diff_sum_velocity_ = brake_diff_sum_velocity_ = 0;
		accel_e_prev_acceleration_ = brake_e_prev_acceleration_ = accel_diff_sum_acceleration_ = brake_diff_sum_acceleration_ = 0;
		accel_e_prev_distance_ = brake_e_prev_distance_ = accel_diff_sum_distance_ = brake_diff_sum_distance_ = 0;
		steer_e_prev_distance_ = steer_diff_sum_distance_ = 0;
		stroke_prev_ = stop_stroke_prev_ = acc_stroke;
		stroke_state_mode_ = PID_params::STROKE_STATE_MODE_ACCEL_;
	}

	void clear_diff_velocity()
	{
		accel_diff_sum_velocity_ = brake_diff_sum_velocity_ = 0;
	}

	void clear_diff_acceleration()
	{
		accel_diff_sum_acceleration_ = brake_diff_sum_acceleration_ = 0;
	}

	void clear_diff_distance()
	{
		accel_diff_sum_distance_ = brake_diff_sum_distance_ = 0;
	}

	void clear_steer_diff_distance()
	{
		steer_diff_sum_distance_ = 0;
	}

	double get_accel_e_prev_velocity() {return accel_e_prev_velocity_;}
	double get_accel_e_prev_acceleration_() {return accel_e_prev_acceleration_;}
	double get_accel_e_prev_distance() {return accel_e_prev_distance_;}
	double get_brake_e_prev_velocity() {return brake_e_prev_velocity_;}
	double get_brake_e_prev_acceleration() {return brake_e_prev_acceleration_;}
	double get_brake_e_prev_distance() {return brake_e_prev_distance_;}
	double get_steer_e_prev_distance() {return steer_e_prev_distance_;}
	double get_acclel_diff_sum_velocity() {return accel_diff_sum_velocity_;}
	double get_acclel_diff_sum_acceleration() {return accel_diff_sum_acceleration_;}
	double get_acclel_diff_sum_distance() {return accel_diff_sum_distance_;}
	double get_brake_diff_sum_velocity() {return brake_diff_sum_velocity_;}
	double get_brake_diff_sum_acceleration() {return brake_diff_sum_acceleration_;}
	double get_brake_diff_sum_distance() {return brake_diff_sum_distance_;}
	double get_steer_diff_sum_distance() {return steer_diff_sum_distance_;}
	double get_stop_stroke_prev() {return stop_stroke_prev_;}
	double get_stroke_prev() {return stroke_prev_;}
	int get_stroke_state_mode_() {return stroke_state_mode_;}

	void set_accel_e_prev_velocity(double val) {accel_e_prev_velocity_ = val;}
	void set_accel_e_prev_acceleration(double val) {accel_e_prev_acceleration_ = val;}
	void set_accel_e_prev_distance(double val) {accel_e_prev_distance_ = val;}
	void set_brake_e_prev_velocity(double val) {brake_e_prev_velocity_ = val;}
	void set_brake_e_prev_acceleration(double val) {brake_e_prev_acceleration_ = val;}
	void set_brake_e_prev_distance(double val) {brake_e_prev_distance_ = val;}
	void set_steer_e_prev_distance(double val) {steer_e_prev_distance_ = val;}
	void plus_accel_diff_sum_velocity(double val) {accel_diff_sum_velocity_ += val;}
	void plus_accel_diff_sum_acceleration(double val) {accel_diff_sum_acceleration_ += val;}
	void plus_accel_diff_sum_distance(double val) {accel_diff_sum_distance_ += val;}
	void plus_brake_diff_sum_velocity(double val) {brake_diff_sum_velocity_ += val;}
	void plus_brake_diff_sum_acceleration(double val) {brake_diff_sum_acceleration_ += val;}
	void plus_brake_diff_sum_distance(double val) {brake_diff_sum_distance_ += val;}
	void plus_steer_diff_sum_distance(double val) {steer_diff_sum_distance_ += val;}
	void set_stop_stroke_prev(double val) {stop_stroke_prev_ = val;}
	void set_stroke_prev(double val) {stroke_prev_ = val;}
	void set_stroke_state_mode_(int val) {stroke_state_mode_ = val;}
};

enum class EControl
{
  KEEP = -1,
  STOP = 1,
  STOPLINE = 2,
  DECELERATE = 3,
  OTHERS = 4,
};

class waypoint_param_geter
{
public:
	char automatic_door_;
	bool automatic_door_taiki_;

	waypoint_param_geter()
	{
		automatic_door_ = 0;
		automatic_door_taiki_ = false;
	}
};

struct LIMIT_ANGLE_FROM_VELOCITY_STRUCT
{
	double velocity;
	double limit_angle_top;
	double limit_angle_bottom;
};

class LIMIT_ANGLE_FROM_VELOCITY_CLASS
{
private:
	std::vector<LIMIT_ANGLE_FROM_VELOCITY_STRUCT> lafvs;
public:
	LIMIT_ANGLE_FROM_VELOCITY_CLASS()
	{
		lafvs.clear();
		LIMIT_ANGLE_FROM_VELOCITY_STRUCT data={0,700,-700};
		lafvs.push_back(data);
	}

	void add(LIMIT_ANGLE_FROM_VELOCITY_STRUCT data) {lafvs.push_back(data);}

	const LIMIT_ANGLE_FROM_VELOCITY_STRUCT getLimit(double vel)
	{
		for(int i=0;i<lafvs.size()-1;i++)
		{
			double bottom = lafvs[i].velocity;
			double top = lafvs[i+1].velocity;
			std::cout << "velocity_range : " << top << "," << vel << "," << bottom << std::endl;
			if(vel >= bottom && vel < top)
			{
				return lafvs[i+1];
			}
		}
//std::cout << "aaaaaaaaaaaaa\n";
		return lafvs[lafvs.size()-1];
	}
};

class kvaser_can_sender
{
private:
	//velcity params
	const short VELOCITY_ZERO_VALUE_ = 132;

	//stroke params
	const short PEDAL_VOLTAGE_CENTER_ = 1024;//1052;//計測値は1025;
	const short PEDAL_DISPLACEMENT_CENTER_ = 1024;//計測値は1029;
	const short PEDAL_VOLTAGE_MAX_ = 161;
	const short PEDAL_DISPLACEMENT_MAX_ = 1161;
	const short PEDAL_VOLTAGE_MIN_ = 1533;
	const short PEDAL_DISPLACEMENT_MIN_ = 849;
	//const short PEDAL_STROKE_CENTER_ = 0;
	//const short PEDAL_STOKE_MAX_ = PEDAL_VOLTAGE_CENTER_ - PEDAL_VOLTAGE_MAX_;
	//const short PEDAL_STROKE_MIN_ = -450;//PEDAL_VOLTAGE_CENTER_ - PEDAL_VOLTAGE_MIN_;
	//const short ACCEL_PEDAL_STROKE_OFFSET_ = 10;
	//const short BRAKE_PEDAL_STROKE_OFFSET_ = -10;
	//const short BRAKE_PEDAL_STOPPING_MED_ = 400;
	//const short BRAKE_PEDAL_STOPPING_MAX_ = 500;

	//steer params
	const double HANDLE_MAX = 675;
	const double WHEEL_MAX = 36;
	const unsigned int STEER_VALUE_MARGIN = 20;

	//vanet params
	//double handle_angle_right_max = 660;
	//double handle_angle_left_max = 670;
	double left_wheel_angle_right_max = 33;
	double right_wheel_angle_right_max = 38;
	double left_wheel_angle_left_max = 39.5;
	double right_wheel_angle_left_max = 33;
	double handle_actual_right_max = 13329;
	double handle_actual_left_max = 14177;
	double handle_offset = 188;

	//liesse params
	double handle_angle_right_max = 730;
	double handle_angle_left_max = 765;
	double wheelrad_to_steering_can_value_left = 20935.4958411006;//20691.8161699557;//20952.8189547718;
	double wheelrad_to_steering_can_value_right = 20791.4464661611;//20802.5331916036;//20961.415734248;
	//double wheelrad_to_steering_can_value_left = 20691.8161699557;
	//double wheelrad_to_steering_can_value_right = 20802.5331916036;
	double angle_magn_right = wheelrad_to_steering_can_value_right / handle_angle_right_max;
	double angle_magn_left = wheelrad_to_steering_can_value_left / handle_angle_left_max;

	//mode params
	const unsigned char MODE_STROKE   = 0x0A;
	const unsigned char MODE_VELOCITY = 0x0B;

	//shift_param
	const static unsigned char SHIFT_P = 0;
	const static unsigned char SHIFT_R = 1;
	const static unsigned char SHIFT_N = 2;
	const static unsigned char SHIFT_D = 3;
	const static unsigned char SHIFT_4 = 4;
	const static unsigned char SHIFT_L = 5;

	//other params
	const unsigned int SEND_DATA_SIZE = 8;

	//error params pid_params.get_stroke_prev();
	const int CONFIG_OK = 0;
	const int ERROR_STROKE_MAX_MIN_INCONSISTENCY = 1;
	const int ERROR_STROKE_CENTER_INCONSISTENCY = 2;
	const int ERROR_STROKE_STOPPING_MED_INCONSISTENCY = 3;
	const int ERROR_STROKE_ACCEL_STROKE_INCONSISTENCY = 4;
	const int ERROR_STROKE_BRAKE_STROKE_INCONSISTENCY = 5;
	int config_result;

	//use velocity topic
	const static int USE_VELOCITY_CAN = 0;
	const static int USE_VELOCITY_TWIST = 1;

	//ues acceleration topic
	const static int USE_ACCELERATION_TWIST1 = 0;
	const static int USE_ACCELERATION_TWIST2 = 1;
	const static int USE_ACCELERATION_IMU = 2;

	//safety
	const LIMIT_ANGLE_FROM_VELOCITY_STRUCT limit10 = {10,680,-680}, limit15 = {15,360,-360}, limit20 = {20,180,-180}, limit30={30,90,-90}, limit40={40,45,-45};
	LIMIT_ANGLE_FROM_VELOCITY_CLASS lafvc_;
	bool dengerStopFlag = false;//自動運転が失敗しそうな場に止めるフラグ

	ros::Publisher pub_microbus_can_sender_status_, pub_acceleration_write_, pub_estimate_stopper_distance_;
	ros::Publisher pub_localizer_match_stat_, pub_stroke_routine_, pub_vehicle_status_, pub_velocity_param_;

	ros::NodeHandle nh_, private_nh_;
	ros::Subscriber sub_microbus_drive_mode_, sub_microbus_steer_mode_, sub_twist_cmd_;
	ros::Subscriber sub_microbus_can_501_, sub_microbus_can_502_, sub_microbus_can_503_;
	//ros::Subscriber sub_current_pose_, sub_current_velocity_;
	ros::Subscriber sub_emergency_reset_, sub_stroke_mode_, sub_velocity_mode_, sub_drive_control_;
	ros::Subscriber sub_input_steer_flag_, sub_input_drive_flag_, sub_input_steer_value_, sub_input_drive_value_;
	ros::Subscriber sub_waypoint_param_, sub_waypoints_, sub_position_checker_, sub_config_microbus_can_;
	ros::Subscriber sub_shift_auto_, sub_shift_position_;
	ros::Subscriber sub_emergency_stop_;
	ros::Subscriber sub_light_high_, sub_light_low_, sub_light_small_;
	ros::Subscriber sub_blinker_right_, sub_blinker_left_, sub_blinker_stop_;
	ros::Subscriber sub_automatic_door_, sub_drive_clutch_, sub_steer_clutch_;
	ros::Subscriber sub_econtrol_, sub_obtracle_waypoint_, sub_stopper_distance_;
	ros::Subscriber sub_lidar_detector_objects_, sub_imu_, sub_gnss_standard_deviation_;
	ros::Subscriber sub_ndt_stat_string, sub_gnss_stat_, sub_ndt_pose_, sub_gnss_pose_, sub_ndt_stat_, sub_ndt_reliability_, sub_ekf_pose_;
	ros::Subscriber sub_difference_to_waypoint_distance_, sub_difference_to_waypoint_distance_ndt_, sub_difference_to_waypoint_distance_gnss_, sub_difference_to_waypoint_distance_ekf_;
	ros::Subscriber sub_localizer_select_num_, sub_config_localizer_switch_, sub_interface_lock_;//sub_interface_config_;
	ros::Subscriber sub_ekf_covariance_, sub_use_safety_localizer_, sub_config_current_velocity_conversion_;
	ros::Subscriber sub_cruse_velocity_, sub_mobileye_frame_, sub_mobileye_obstacle_data_, sub_temporary_fixed_velocity_;


	message_filters::Subscriber<geometry_msgs::TwistStamped> *sub_current_velocity_;
	message_filters::Subscriber<geometry_msgs::PoseStamped> *sub_current_pose_;
	message_filters::Synchronizer<TwistPoseSync> *sync_twist_pose_;

	KVASER_CAN kc;
	bool flag_drive_mode_, flag_steer_mode_;
	bool input_drive_mode_, input_steer_mode_;
	autoware_config_msgs::ConfigMicroBusCan setting_;
	autoware_config_msgs::ConfigLocalizerSwitchFusion config_localizer_switch_;
	//autoware_config_msgs::ConfigMicrobusInterface interface_config_;
	unsigned char drive_control_mode_;
	autoware_can_msgs::MicroBusCan501 can_receive_501_;
	autoware_can_msgs::MicroBusCan502 can_receive_502_;
	autoware_can_msgs::MicroBusCan503 can_receive_503_;
	geometry_msgs::TwistStamped current_velocity_;
	geometry_msgs::PoseStamped current_pose_;
	sensor_msgs::Imu imu_;
	double acceleration1_twist_, acceleration2_twist_, jurk1_twist_, jurk2_twist_;
	std::vector<double> acceleration_vec1_, acceleration_vec2_;
	autoware_msgs::VehicleCmd twist_;
	short input_steer_, input_drive_;
	short pedal_;
	bool shift_auto_;
	unsigned char shift_position_, drive_clutch_, steer_clutch_, automatic_door_;
	unsigned char emergency_stop_;
	bool light_high_, light_low_, light_small_;
	bool blinker_right_, blinker_left_, blinker_stop_, blinker_param_sender_;
	EControl econtrol;
	int obstracle_waypoint_;
	double stopper_distance_;
	autoware_msgs::WaypointParam waypoint_param_;
	PID_params pid_params;
	int use_velocity_data_, use_acceleration_data_;
	autoware_msgs::PositionChecker position_checker_;
	bool angle_limit_over_;
	double steer_correction_;
	std::string ndt_stat_string_;
	geometry_msgs::PoseStamped ndt_pose_, gnss_pose_, ekf_pose_;
	unsigned char gnss_stat_;
	autoware_msgs::DifferenceToWaypointDistance difference_toWaypoint_distance_, difference_toWaypoint_distance_ndt_, difference_toWaypoint_distance_gnss_, difference_toWaypoint_distance_ekf_;
	autoware_msgs::GnssStandardDeviation gnss_deviation_;
	autoware_msgs::NDTStat ndt_stat_;
	double ndt_reliability_;
	int localizer_select_num_;
	ros::Time automatic_door_time_;
	ros::Time blinker_right_time_, blinker_left_time_, blinker_stop_time_;
	ros::Time drive_clutch_timer_, steer_clutch_timer_;
	double waypoint_id_ = -1;
	tf::Quaternion waypoint_orientation_;
	double ndt_gnss_angle_, waypoint_angle_;
	tf::Quaternion waypoint_localizer_angle_;
	waypoint_param_geter wpg_;
	double accel_avoidance_distance_min_, stop_stroke_max_;
	bool in_accel_mode_, in_brake_mode_;
	std_msgs::String routine_;
	bool use_stopper_distance_, interface_lock_;
	unsigned int loop_counter_;
	geometry_msgs::PoseWithCovarianceStamped ekf_covariance_;
	int ndt_warning_count_;
	bool use_safety_localizer_;
	autoware_config_msgs::ConfigCurrentVelocityConversion config_current_velocity_conversion_;
	double cruse_velocity_;
	mobileye_560_660_msgs::AftermarketLane mobileye_lane_;
	mobileye_560_660_msgs::ObstacleData mobileye_obstacle_data_;
	double temporary_fixed_velocity_;
	double send_step_;

	void callbackTemporaryFixedVelocity(const std_msgs::Float64::ConstPtr &msg)
	{
		temporary_fixed_velocity_ = msg->data;
	}

	bool checkMobileyeObstacleStop(ros::Time nowtime)
	{
		ros::Duration t = nowtime - mobileye_obstacle_data_.header.stamp;
		ros::Duration th = ros::Duration(1);
		std::cout << "mobs : " << t << "," << th << "," << (int)mobileye_obstacle_data_.obstacle_status << std::endl;
		if(t < th)
		{
			switch(mobileye_obstacle_data_.obstacle_status)
			{
				case 0://undefind
				case 1://standing
				case 2://stoped
				case 5://parked
					std::cout << "mobs" << std::endl;
					return true;
				default:
					return false;
			}
		}
		else return false;
	}

	void callbackMobileyeObstacleData(const mobileye_560_660_msgs::ObstacleData::ConstPtr &msg)
	{
		mobileye_obstacle_data_ = *msg;
	}

	const bool getMessage_bool(const unsigned char *buf, unsigned int bit)
	{
		unsigned long long mask=1;
		mask<<=bit;
		unsigned long long *msgL=(unsigned long long)buf;
		if((*msgL & mask)) return true;
		else return false;
	}

	template<typename T>
	const T getMessage_bit(const unsigned char *buf, const unsigned int lowBit, const unsigned int highBit)
	{
		const unsigned int maxBitSize=sizeof(unsigned long long)*8;
		unsigned long long *msgL=(unsigned long long)buf;
		unsigned long long val=(*msgL)<<maxBitSize-highBit-1;
		unsigned int lowPos=lowBit+(maxBitSize-highBit-1);
		val>>=lowPos;
		return (T)val;
	}

	void callbackMobileyeCan(const can_msgs::Frame &frame)
	{
		switch(frame.id)
		{
		case 0x669:
			{
				if(frame.is_error == false && frame.dlc == 8)
				{
					const unsigned char *buf = (unsigned char*)frame.data.data();
					//Lane type
					mobileye_lane_.lane_type_left = getMessage_bit<unsigned char>(&buf[0], 4, 7);
					mobileye_lane_.lane_type_right = getMessage_bit<unsigned char>(&buf[5], 4, 7);
					//ldw_available
					mobileye_lane_.ldw_available_left = getMessage_bool(&buf[0], 2);
					mobileye_lane_.ldw_available_right = getMessage_bool(&buf[5], 2);
					//lane_confidence
					mobileye_lane_.lane_confidence_left = getMessage_bit<unsigned char>(&buf[0], 0, 1);
					mobileye_lane_.lane_confidence_right = getMessage_bit<unsigned char>(&buf[5], 0, 1);
					//distance_to lane
					int16_t distL, distR;
					unsigned char* distL_p = (unsigned char*)&distL;
					distL_p[1] = getMessage_bit<unsigned char>(&buf[2], 4, 7);
					distL_p[0] = getMessage_bit<unsigned char>(&buf[2], 0, 3) << 4;
					distL_p[0] |= getMessage_bit<unsigned char>(&buf[1], 4, 7);
					if(distL_p[1] & 0x8)//12bitのマイナスか
					{
						distL--;
						distL = ~distL;
						distL_p[1] &= 0x0F;
						distL = -distL;
					}
					mobileye_lane_.distance_to_left_lane = distL * 0.02;
					//std::cout << "distL : " << (int)distL << std::endl;
					unsigned char* distR_p = (unsigned char*)&distR;
					distR_p[1] = getMessage_bit<unsigned char>(&buf[7], 4, 7);
					distR_p[0] = getMessage_bit<unsigned char>(&buf[7], 0, 3) << 4;
					distR_p[0] |= getMessage_bit<unsigned char>(&buf[6], 4, 7);
					if(distR_p[1] & 0x8)//12bitのマイナスか
					{
						distR--;
						distR = ~distR;
						distR_p[1] &= 0x0F;
						distR = -distR;
					}
					mobileye_lane_.distance_to_right_lane = distR * 0.02;
					//std::cout << "distR : " << (int)distR << std::endl;
				}
				break;
			}
		}
	}

	void callbackCurrentVelocityConversion(const autoware_config_msgs::ConfigCurrentVelocityConversion::ConstPtr &msg)
	{
		config_current_velocity_conversion_ = *msg;
	}

	void callbackCruseVelocity(const std_msgs::Float64::ConstPtr &msg)
	{
		cruse_velocity_ = msg->data;
	}

	void callbackUseSafetyLocalizer(const std_msgs::Bool::ConstPtr &msg)
	{
		use_safety_localizer_ = msg->data;
		//steer_clutch_ = false;
		//std::cout << "use_safety : " << (int)use_safety_localizer_ << std::endl;
	}

	void callbackInterfaceLock(const std_msgs::BoolConstPtr &msg)
	{
		interface_lock_ = msg->data;
	}

	const int LOCALIZER_SELECT_NDT = 0;
	const int LOCALIZER_SELECT_GNSS = 1;
	void callbackLocalizerSelectNum(const std_msgs::Int32::ConstPtr msg)
	{
		localizer_select_num_ = msg->data;

		if(localizer_select_num_ < 0 && use_safety_localizer_ == true)
		{
			if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_clutch_ = false;
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				steer_clutch_ = false;
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			std::cout << "Denger! localizer change error" << std::endl;
			std::stringstream safety_error_message;
			safety_error_message << "localizer change error";
			publishStatus(safety_error_message.str());
			//system("aplay -D plughw:PCH /home/autoware/one33.wav");
			can_send();
		}
	}

	void waypointDistanceCheck(const autoware_msgs::DifferenceToWaypointDistance::ConstPtr &msg, std::string pose_name)
	{
		if((fabs(msg->baselink_distance) > setting_.check_distance_th ||//setting_.difference_to_waypoint_distance ||
			fabs(msg->baselink_angular) > setting_.check_angular_th) &&
			use_safety_localizer_ == true)
		{
			if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_clutch_ = false;
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				steer_clutch_ = false;
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			std::cout << "Denger! " << pose_name << " : distance : " << msg->baselink_distance << "  angular : " << msg->baselink_angular << std::endl;
			std::stringstream safety_error_message;
			safety_error_message << pose_name << "\ndistance," << msg->baselink_distance << "\nangular," << msg->baselink_angular;
			publishStatus(safety_error_message.str());
			//system("aplay -D plughw:PCH /home/autoware/one33.wav");
			can_send();
		}
	}

	void callbackDifferenceToWaypointDistance(const autoware_msgs::DifferenceToWaypointDistance::ConstPtr &msg)
	{
		waypointDistanceCheck(msg, std::string("current"));
		difference_toWaypoint_distance_ = *msg;
	}

	void callbackDifferenceToWaypointDistanceNdt(const autoware_msgs::DifferenceToWaypointDistance::ConstPtr &msg)
	{
		if(config_localizer_switch_.localizer_check == 0 || config_localizer_switch_.localizer_check == 2)
			waypointDistanceCheck(msg, std::string("ndt"));
		difference_toWaypoint_distance_ndt_ = *msg;
	}

	void callbackDifferenceToWaypointDistanceGnss(const autoware_msgs::DifferenceToWaypointDistance::ConstPtr &msg)
	{
		if(config_localizer_switch_.localizer_check == 1 || config_localizer_switch_.localizer_check == 2)
			waypointDistanceCheck(msg, std::string("gnss"));
		difference_toWaypoint_distance_gnss_ = *msg;
	}

	void callbackDifferenceToWaypointDistanceEkf(const autoware_msgs::DifferenceToWaypointDistance::ConstPtr &msg)
	{
		//if(config_localizer_switch_.localizer_check == 1 || config_localizer_switch_.localizer_check == 2)
		//	waypointDistanceCheck(msg, std::string("gnss"));
		difference_toWaypoint_distance_ekf_ = *msg;
	}
	
	void NdtGnssCheck()
	{
		bool flag = true;
		if(localizer_select_num_ == 0 && gnss_stat_ != 3) flag = false;
		//if(ndt_stat_string != "NDT_OK") flag = false;

		double ndtx = ndt_pose_.pose.position.x;
		double ndty = ndt_pose_.pose.position.y;
		double gnssx = gnss_pose_.pose.position.x;
		double gnssy = gnss_pose_.pose.position.y; 
		double diff_x = ndtx - gnssx;
		double diff_y = ndty - gnssy;
		double distance = sqrt(diff_x * diff_x + diff_y* diff_y);
		/*if(distance > 8)
		{
			if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_clutch_ = false;
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				steer_clutch_ = false;
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			std::cout << "Denger! Ndt Gnss Distance : " << distance << std::endl;
			std::stringstream safety_error_message;
			safety_error_message << "Ndt Gnss Distance" << distance;
			publishStatus(safety_error_message.str());
			can_send();
			flag = false;
		}*/

		/*diff_x /= distance;
		diff_y /= distance;
		ndt_gnss_angle_ = atan2(diff_x,diff_y);//map座標系はｘが上
		tf::Quaternion localize_qua;
		localize_qua.setRPY(0,0,ndt_gnss_angle_);
		waypoint_localizer_angle_ = waypoint_orientation_ * localize_qua.inverse();
		//if(distance >= 0.5) flag = false;//0.5
		double wla_roll, wla_pitch, wla_yaw;
		tf::Matrix3x3 mat_waypoint_localizer_angle_(waypoint_localizer_angle_);
		mat_waypoint_localizer_angle_.getRPY(wla_roll, wla_pitch, wla_yaw);
		double waypoint_localizer_angle_deg = wla_yaw * 180.0 / M_PI;*/

		autoware_msgs::LocalizerMatchStat lms;
		lms.header.stamp = ros::Time::now();

		std::string gnss_stat_string = (gnss_stat_ = 3) ? "GNSS_OK" : "GNSS_ERROR";
		std::cout << "stat : " << ndt_stat_string_ << "," << gnss_stat_string << std::endl;

		if(config_localizer_switch_.localizer_check == 2)
		{
			if(fabs(difference_toWaypoint_distance_gnss_.baselink_distance - difference_toWaypoint_distance_ndt_.baselink_distance) > setting_.ndt_gnss_max_distance_limit &&
			   use_safety_localizer_ == true)
			{
				if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
					drive_clutch_ = false;
				if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
					steer_clutch_ = false;
				//flag_drive_mode_ = false;
				//flag_steer_mode_ = false;
				shift_auto_ = false;
				std::cout << "Denger! difference_toWaypoint_distance_gnss : " << difference_toWaypoint_distance_ndt_.front_baselink_distance << "," << difference_toWaypoint_distance_gnss_.baselink_distance<< std::endl;
				std::stringstream safety_error_message;
				safety_error_message << "not difference_toWaypoint\ndistance_gnss\n" << difference_toWaypoint_distance_ndt_.front_baselink_distance << "," << difference_toWaypoint_distance_gnss_.baselink_distance;
				publishStatus(safety_error_message.str());
				//system("aplay -D plughw:PCH /home/autoware/one33.wav");
				//can_send();
			}
		}
		/*if(setting_.ndt_gnss_min_distance_limit <= distance)
		{
			ndt_gnss_difference_stat = true;
		}
		else if(setting_.ndt_gnss_max_distance_limit <= distance)
		{
			if( waypoint_localizer_angle_deg > 180 - setting_.ndt_gnss_angle_limit 
			   || waypoint_localizer_angle_deg < -180 +  setting_.ndt_gnss_angle_limit)
			   {
					ndt_gnss_difference_stat = true;
			   }
		}*/

		lms.localizer_stat = false;
		switch(config_localizer_switch_.localizer_check)
		{
			case 0://ndt only
			{
				if(ndt_stat_string_ == "NDT_OK")
				{
					ndt_warning_count_ = 0;
					lms.localizer_stat = true;
				}
				if(ndt_stat_string_ == "NDT_WARNING")
				{
					ndt_warning_count_++;
					if(ndt_warning_count_ < 50) lms.localizer_stat = true;
				}
				break;
			}
			case 1://gnss only
			{
				if(gnss_stat_string == "GNSS_OK") lms.localizer_stat = true;
				break;
			}
			case 2://ndt and gnss
			{
				if(ndt_stat_string_ == "NDT_OK" && gnss_stat_string == "GNSS_OK") lms.localizer_stat = true;
				break;
			}
		}

		/*if(localizer_select_num_ == 1 && ndt_stat_string_ == "NDT_OK" && gnss_stat_string == "GNSS_OK")// ndt_gnss_difference_stat)
		{
			lms.localizer_stat = true;
			//lms.localizer_distance = distance;
		}
		else if(localizer_select_num_ == 0 && ndt_stat_string_ == "NDT_OK")
		{
			lms.localizer_stat = true;
		}
		else*/
		if(lms.localizer_stat == false && use_safety_localizer_ == true)
		{
			if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_clutch_ = false;
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				steer_clutch_ = false;
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			std::cout << "Denger! not OK : " << ndt_stat_string_ << "," << gnss_stat_string << std::endl;
			std::stringstream safety_error_message;
			safety_error_message << "not OK : " << ndt_stat_string_ << "," << gnss_stat_string;
			publishStatus(safety_error_message.str());
			//system("aplay -D plughw:PCH /home/autoware/one33.wav");
			//can_send();
			//lms.localizer_stat = false;
			//lms.localizer_distance = distance;
		}

		pub_localizer_match_stat_.publish(lms);

		/*if(flag == false)
		{
			if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_clutch_ = false;
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				steer_clutch_ = false;
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			std::cout << "Denger! Ndt Gnss check : " << std::endl;
			std::stringstream safety_error_message;
			safety_error_message << "Ndt Gnss error : ";
			publishStatus(safety_error_message.str());
			can_send();
		}*/

		if(mobileye_lane_.lane_type_left != mobileye_560_660_msgs::AftermarketLane::LANE_TYPE_NONE &&
           mobileye_lane_.lane_confidence_left >= 2 && use_safety_localizer_ == true && setting_.use_lane_left == true)
		{
			if(mobileye_lane_.distance_to_left_lane < setting_.lane_th_left)
			{
				if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
					drive_clutch_ = false;
				if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
					steer_clutch_ = false;
				shift_auto_ = false;
				std::cout << "Denger! left lane : " << "," << mobileye_lane_.distance_to_left_lane << std::endl;
				std::stringstream safety_error_message;
				safety_error_message << "left lane : " << setting_.lane_th_left << "," << mobileye_lane_.distance_to_left_lane;
				publishStatus(safety_error_message.str());
			}
		}

		if(mobileye_lane_.lane_type_right != mobileye_560_660_msgs::AftermarketLane::LANE_TYPE_NONE &&
           mobileye_lane_.lane_confidence_right >= 2 && use_safety_localizer_ == true && setting_.use_lane_right == true)
		{
			if(mobileye_lane_.distance_to_right_lane > setting_.lane_th_right)
			{
				if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
					drive_clutch_ = false;
				if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
					steer_clutch_ = false;
				shift_auto_ = false;
				std::cout << "Denger! right lane : " << "," << mobileye_lane_.distance_to_right_lane << std::endl;
				std::stringstream safety_error_message;
				safety_error_message << "right lane : " << setting_.lane_th_right << "," << mobileye_lane_.distance_to_right_lane;
				publishStatus(safety_error_message.str());
			}
		}
	}

	void callbackNdtPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		ndt_pose_ = *msg;
		//NdtGnssCheck();
	}

	void callbackGnssPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		gnss_pose_ = *msg;
		//NdtGnssCheck();
	}

	void callbackEkfPose(const geometry_msgs::PoseStamped::ConstPtr &msg)
	{
		ekf_pose_ = *msg;
		//NdtGnssCheck();
	}

	void callbackEkfCovariance(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
	{
		ekf_covariance_ = *msg;
	}

	void callbackNdtStatString(const std_msgs::String::ConstPtr &msg)
	{
		ndt_stat_string_ = msg->data;
	}

	void callbackNdtStat(const autoware_msgs::NDTStat::ConstPtr &msg)
	{
		ndt_stat_ = *msg;
	}

	void callbackNdtReliability(const std_msgs::Float32::ConstPtr &msg)
	{
		ndt_reliability_ = msg->data;
	}

	void callbackGnssStat(const std_msgs::UInt8::ConstPtr &msg)
	{
		gnss_stat_ = msg->data;
	}

	void callbackGnssStandardDeviation(const autoware_msgs::GnssStandardDeviation::ConstPtr &msg)
	{
		std::cout << "gnss_lat : " << msg->lat_std << "," << setting_.gnss_lat_limit << std::endl;
		std::cout << "gnss_lon : " << msg->lon_std << "," << setting_.gnss_lon_limit << std::endl;
		std::cout << "gnss_alt : " << msg->alt_std << "," << setting_.gnss_alt_limit << std::endl;

		if((config_localizer_switch_.localizer_check == 1 || config_localizer_switch_.localizer_check == 2) && localizer_select_num_ == 1)
		{
			if((msg->lat_std > setting_.gnss_lat_limit ||
		        msg->lon_std > setting_.gnss_lon_limit ||
		        msg->alt_std > setting_.gnss_alt_limit && localizer_select_num_ == LOCALIZER_SELECT_GNSS) &&
				use_safety_localizer_ == true)
			{
				if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
					drive_clutch_ = false;
				if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
					steer_clutch_ = false;
				//flag_drive_mode_ = false;
				//flag_steer_mode_ = false;
				shift_auto_ = false;
				std::cout << "Denger! Gnss deviation limit over : " << msg->lat_std << "," << msg->lon_std << "," << msg->alt_std << std::endl;
				std::stringstream safety_error_message;
				safety_error_message << "Gnss deviation error : ";// << msg->lat_std << "," << msg->lon_std << "," << msg->alt_std;
				publishStatus(safety_error_message.str());
				//system("aplay -D plughw:PCH /home/autoware/one33.wav");
				can_send();
			}
		}
		gnss_deviation_ = *msg;
	}

	void callbackImu(const sensor_msgs::Imu::ConstPtr msg)
	{
		imu_ = *msg;
	}

	void callbackLidarDetectorObjects (const autoware_msgs::DetectedObjectArray::ConstPtr &msg)
	{
		std::cout << "lidar detector : " << std::endl;
		std::cout << "ldo obj size : " << msg->objects.size() << std::endl;
		for(int obj_i=0; obj_i<msg->objects.size(); obj_i++)
		{
			int id = msg->objects[obj_i].id;
			geometry_msgs::Pose pose = msg->objects[obj_i].pose;
			std::cout << "ldo " << id << " pose x : " << pose.position.x << std::endl;
			std::cout << "ldo " << id << " pose y : " << pose.position.y << std::endl;
			std::cout << "ldo " << id << " pose z : " << pose.position.z << std::endl;
			std::cout << "ldo " << id << " orie x : " << pose.orientation.x << std::endl;
			std::cout << "ldo " << id << " orie y : " << pose.orientation.y << std::endl;
			std::cout << "ldo " << id << " orie z : " << pose.orientation.z << std::endl;
			std::cout << "ldo " << id << " orie w : " << pose.orientation.w << std::endl;

			geometry_msgs::Twist velocity = msg->objects[obj_i].velocity;
			std::cout << "ldo " << id << " linear x : " << velocity.linear.x << std::endl;
			std::cout << "ldo " << id << " linear y : " << velocity.linear.y << std::endl;
			std::cout << "ldo " << id << " linear z : " << velocity.linear.z << std::endl;
			std::cout << "ldo " << id << " ang x : " << velocity.angular.x << std::endl;
			std::cout << "ldo " << id << " ang y : " << velocity.angular.y << std::endl;
			std::cout << "ldo " << id << " ang z : " << velocity.angular.z << std::endl;
		}
	}

	void callbackStopperDistance(const std_msgs::Float64::ConstPtr &msg)
	{
		stopper_distance_ = msg->data;
		std::cout << "stopper distance : " << stopper_distance_ << std::endl;
	}

	void callbackEmergencyStop(const std_msgs::UInt8::ConstPtr &msg)
	{
		emergency_stop_ = msg->data;
		std::cout << "emergency stop : " << (int)emergency_stop_ << std::endl;
	}

	void callbackObstracleWaypoint(const std_msgs::Int32::ConstPtr &msg)
	{
		obstracle_waypoint_ = msg->data;
	}

	void callbackEControl(const std_msgs::Int8::ConstPtr &msg)
	{
		econtrol = (EControl)msg->data;
	}

	void callbackEmergencyReset(const std_msgs::Empty::ConstPtr &msg)
	{
		std::cout << "sub Emergency" << std::endl;
		char buf[SEND_DATA_SIZE] = {0,0,0,0,0,0,0,0};
		buf[0] = 0x55;
		kc.write(0x100, buf, SEND_DATA_SIZE);
		ros::Rate rate(1);
		rate.sleep();
		buf[0] = 0x00;
		kc.write(0x100, buf, SEND_DATA_SIZE);
	}

	void callbackDModeSend(const std_msgs::Bool::ConstPtr &msg)
	{
		std::string flag = (msg->data) ? "on" : "off";
		std::cout << "sub DMode : " << flag << std::endl;
		flag_drive_mode_ = msg->data;
	}

	void callbackSModeSend(const std_msgs::Bool::ConstPtr &msg)
	{
		std::string flag = (msg->data) ? "on" : "off";
		std::cout << "sub SMode : " << flag << std::endl;
		flag_steer_mode_ = msg->data;
	}

	void callbackMicrobusCan501(const autoware_can_msgs::MicroBusCan501::ConstPtr &msg)
	{
		std::cout << "sub can_501" << std::endl;
		can_receive_501_ = *msg;
	}

	void callbackMicrobusCan502(const autoware_can_msgs::MicroBusCan502::ConstPtr &msg)
	{
		std::cout << "sub can_502" << std::endl;
		if(msg->clutch==true && can_receive_502_.clutch==false)
		{
			input_steer_mode_ = false; //std::cout << "aaa" << std::endl;
			std::string safety_error_message = "";
			publishStatus(safety_error_message);
		}
		if(msg->clutch==false && can_receive_502_.clutch==true)
		{
			input_steer_mode_ = true;
			std::string safety_error_message = "";
			publishStatus(safety_error_message);
		}

		can_receive_502_ = *msg;
	}

	void callbackMicrobusCan503(const autoware_can_msgs::MicroBusCan503::ConstPtr &msg)
	{
		std::cout << "sub can_503" << std::endl;
		if(msg->clutch==true && can_receive_503_.clutch==false)
		{
			//drive_control_mode_ = MODE_VELOCITY;
			shift_auto_ = true;
			input_drive_mode_ = false;
			std::string safety_error_message = "";
			publishStatus(safety_error_message);
		}
		if(msg->clutch==false && can_receive_503_.clutch==true)
		{
			//drive_control_mode_ = MODE_STROKE;
			shift_auto_ = false;
			input_drive_mode_ = true;
			std::string safety_error_message = "";
			publishStatus(safety_error_message);
		}
		can_receive_503_ = *msg;
	}

	/*void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg)
	{
		std::cout << "current velocity : " << msg->twist.linear.x << std::endl;
		ros::Duration rostime = msg->header.stamp - current_velocity_.header.stamp;
		double td = rostime.sec + rostime.nsec * 1E-9;
		acceleration_ = (msg->twist.linear.x - current_velocity_.twist.linear.x) / td;
		std::cout << "acceleration," << acceleration_ << std::endl;
		current_velocity_ = *msg;
	}*/

	void TwistPoseCallback(const geometry_msgs::TwistStampedConstPtr &twist_msg,
	                       const geometry_msgs::PoseStampedConstPtr &pose_msg)
	{
		std::cout << "current velocity : " << twist_msg->twist.linear.x << std::endl;
		std::cout << "current pose : "     << pose_msg->pose.position.x << "," << pose_msg->pose.position.y << std::endl;

		ros::Duration rostime = twist_msg->header.stamp - current_velocity_.header.stamp;
		double td = rostime.sec + rostime.nsec * 1E-9;
		std::cout << "acc_time," << td << std::endl;

		// v^2 - v0^2 = 2ax
		const double x =
		    std::hypot(pose_msg->pose.position.x - current_pose_.pose.position.x, pose_msg->pose.position.y - current_pose_.pose.position.y);
		const double v0 = current_velocity_.twist.linear.x;
		const double v = twist_msg->twist.linear.x;
		double v_sa = v * v - v0 * v0;
		double acc = 0;
		//if(x >= 0.01) acc = (v_sa) / (2 * x);
		acceleration_vec1_.insert(acceleration_vec1_.begin(), acc);
		if(acceleration_vec1_.size() > 3) acceleration_vec1_.resize(3);
		acc = 0;
		int vec_cou;
		for(vec_cou=0; vec_cou<acceleration_vec1_.size(); vec_cou++)
		{
			acc += acceleration_vec1_[vec_cou];
		}
		acc /= vec_cou;
		double jurk = (acc - acceleration1_twist_) / td;

		double acc2 = (twist_msg->twist.linear.x - current_velocity_.twist.linear.x) / td;
		acceleration_vec2_.insert(acceleration_vec2_.begin(), acc2);
		if(acceleration_vec2_.size() > 3) acceleration_vec2_.resize(3);
		acc2 = 0;
		for(vec_cou=0; vec_cou<acceleration_vec2_.size(); vec_cou++)
		{
			acc2 += acceleration_vec2_[vec_cou];
		}
		acc2 /= vec_cou;
		double jurk2 = (acc2 - acceleration2_twist_) / td;
		std::cout << "acceleration," << acc << "," << acc2 << std::endl;
		double wheel_base = 3.935;
		double tire_angle;
		if(twist_.ctrl_cmd.steering_angle > 0) tire_angle = twist_.ctrl_cmd.steering_angle*wheelrad_to_steering_can_value_left;
		else tire_angle = twist_.ctrl_cmd.steering_angle*wheelrad_to_steering_can_value_right;
		std::stringstream str;
//		str << std::setprecision(10) << waypoint_id_ << "," << pose_msg->pose.position.x << "," << pose_msg->pose.position.y << "," << pose_msg->pose.orientation.z << ",";
//		str << twist_msg->twist.angular.z << "," << twist_msg->twist.linear.x << ",";
//		str << acc << "," << jurk << "," << acc2 << "," << jurk2 << "," << td << ",";
//		str << x << "," << v0 << "," << v << "," << v_sa;
		str << std::setprecision(10) 
			<< waypoint_id_ 
			<< twist_.ctrl_cmd.linear_velocity
			<< cruse_velocity_
			<< "," << twist_msg->twist.linear.x 
			<<"," << acc2 
			<< "," << jurk2 
			<< "," << difference_toWaypoint_distance_.baselink_angular
			<< "," << difference_toWaypoint_distance_.baselink_distance;
		//str << difference_toWaypoint_distance_.front_baselink_distance <<",";
		//str << _steer_pid_control(difference_toWaypoint_distance_.front_baselink_distance) ;
	
		double mps = current_velocity_.twist.linear.x;
		double estimated_stopping_distance = (0 * 0 - mps*mps)/(2.0*acceleration2_twist_);

		std::string gnss_stat_string = (gnss_stat_ == 3) ? "GNSS_OK" : "GNSS_ERROR";
		str << "," << stopper_distance_
			<< "," << estimated_stopping_distance 
			<< "," << ndt_stat_.score 
			<< "," << ndt_reliability_ 
			<< "," << ndt_stat_.exe_time
			<< "," << ndt_stat_string_  //13
			<< "," << gnss_stat_string
			<< "," << gnss_deviation_.lat_std
			<< "," << gnss_deviation_.lon_std
			<< "," << gnss_deviation_.alt_std;
		tf::Quaternion gnss_qua;
		tf::quaternionMsgToTF(gnss_pose_.pose.orientation, gnss_qua);
		tf::Matrix3x3 gnss_mat(gnss_qua);
		double gnss_roll, gnss_pitch, gnss_yaw;
		gnss_mat.getRPY(gnss_roll, gnss_pitch, gnss_yaw);
		str << "," << gnss_roll //18
		    << "," << gnss_pitch 
			<< "," << gnss_yaw;
		double ndtx = ndt_pose_.pose.position.x;
		double ndty = ndt_pose_.pose.position.y;
		
		double gnssx = gnss_pose_.pose.position.x;//23
		double gnssy = gnss_pose_.pose.position.y;

		double ekfx = ekf_pose_.pose.position.x;
		double ekfy = ekf_pose_.pose.position.y;
		double distance = sqrt((ndtx - gnssx) * (ndtx -gnssx) + (ndty - gnssy) * (ndty -gnssy));
		str <<"," <<ndtx 
		    <<"," <<ndty
			<<"," << gnssx
			<<"," << gnssy//28
			<<"," << ekfx
			<<"," << ekfy
			<<"," << ekf_covariance_.pose.covariance[0]
			<<"," << ekf_covariance_.pose.covariance[6*1+1]
			<<"," << distance;
		double roll, pitch, yaw;
		tf::Matrix3x3 wla(waypoint_localizer_angle_);
		wla.getRPY(roll, pitch, yaw);
		str << "," <<  waypoint_angle_ 
			<<"," << ndt_gnss_angle_ 
			<< "," << yaw;


		str << "," << difference_toWaypoint_distance_ndt_.baselink_distance
		    << "," << difference_toWaypoint_distance_gnss_.baselink_distance
			<< "," << difference_toWaypoint_distance_ekf_.baselink_distance;
		str << "," << difference_toWaypoint_distance_ndt_.baselink_distance - difference_toWaypoint_distance_gnss_.baselink_distance;
		str << "," << difference_toWaypoint_distance_ndt_.baselink_distance - difference_toWaypoint_distance_ekf_.baselink_distance;
		str << "," << difference_toWaypoint_distance_ekf_.baselink_distance - difference_toWaypoint_distance_gnss_.baselink_distance;//42

		tf::Quaternion ndt_q = tf::createQuaternionFromYaw(difference_toWaypoint_distance_ndt_.baselink_angular);
		tf::Quaternion ekf_q = tf::createQuaternionFromYaw(difference_toWaypoint_distance_ekf_.baselink_angular);
		tf::Quaternion gnss_q = tf::createQuaternionFromYaw(difference_toWaypoint_distance_gnss_.baselink_angular);
		tf::Quaternion q_ndt_ekf = ndt_q * ekf_q.inverse();
		tf::Quaternion q_ndt_gnss = ndt_q * gnss_q.inverse();
		tf::Quaternion q_ekf_gnss = ekf_q * gnss_q.inverse();
		tf::Matrix3x3 s_ndt_ekf(q_ndt_ekf);
		tf::Matrix3x3 s_ndt_gnss(q_ndt_gnss);
		tf::Matrix3x3 s_ekf_gnss(q_ekf_gnss);
		double getyaw, getroll, getpitch;
		s_ndt_gnss.getRPY(getroll, getpitch, getyaw);
		str << "," << getyaw;
		s_ndt_ekf.getRPY(getroll, getpitch, getyaw);
		str << "," << getyaw;
		s_ekf_gnss.getRPY(getroll, getpitch, getyaw);
		str << "," << getyaw;
		str << "," << (int)can_receive_502_.clutch;
		str << "," << (int)can_receive_503_.clutch;
		str << "," << can_receive_501_.velocity;
		str << "," << can_receive_502_.velocity_actual;//micom_drive_value_;
		str << "," << can_receive_501_.steering_angle;
		str << "," << can_receive_502_.angle_actual;
		str << "," << routine_;
		str << "," << pid_params.get_stroke_prev();
		str << "," << pid_params.get_stop_stroke_prev();
		str << "," << send_step_;

		std_msgs::String aw_msg;
		aw_msg.data = str.str();
		pub_acceleration_write_.publish(aw_msg);

		current_velocity_ = *twist_msg;
		current_pose_ = *pose_msg;
		acceleration1_twist_ = acc;
		acceleration2_twist_ = acc2;
		jurk1_twist_ = jurk;
		jurk2_twist_ = jurk2;

		autoware_can_msgs::MicroBusCanVelocityParam vparam;
		vparam.header.stamp = ros::Time::now();
		vparam.velocity = current_velocity_.twist.linear.x;
		vparam.acceleration = acc2;
		vparam.jurk = jurk2;
		pub_velocity_param_.publish(vparam);
	}

	void callbackTwistCmd(const autoware_msgs::VehicleCmd::ConstPtr &msg)
	{
		std::cout << "sub twist" << std::endl;

		ros::Duration rostime = msg->header.stamp - twist_.header.stamp;
		double time_sa = rostime.sec + rostime.nsec * 1E-9;
		double ws_ave = (wheelrad_to_steering_can_value_left + wheelrad_to_steering_can_value_right) / 2.0;
		double deg = fabs(msg->ctrl_cmd.steering_angle - twist_.ctrl_cmd.steering_angle) * ws_ave * 720.0 / 15000.0;
		double zisoku = msg->ctrl_cmd.linear_velocity * 3.6;

		bool flag = false;

		LIMIT_ANGLE_FROM_VELOCITY_STRUCT limitAngleData = lafvc_.getLimit(zisoku);
		std::cout <<"angle range : " << limitAngleData.limit_angle_bottom << "," << deg << "," << limitAngleData.limit_angle_top << std::endl;

		//double targetAngleTimeVal = fabs(deg - front_deg_)/time_sa;
		std::cout << "time_sa," << time_sa << ",targetAngleTimeVal," << deg << std::endl;
		double deg_th;
		if(zisoku <= 10) deg_th = setting_.steer_speed_limit1;//100;
		else deg_th = setting_.steer_speed_limit2;
		if(deg > deg_th)// && strinf.mode == MODE_PROGRAM)
		{
			if(msg->ctrl_cmd.steering_angle != 0)
			{
				flag = true;
				angle_limit_over_ = true;
				std::cout << "Denger! target angle over : " << deg << std::endl;
			}
			else angle_limit_over_ = false;
		}
		else angle_limit_over_ = false;

		//クラッチが入っている(autoモードである)場合だけエラー判定を出す
		std::stringstream safety_error_message;
		if(flag == true && can_receive_502_.clutch == true && use_safety_localizer_ == true)
		{
			if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
				steer_clutch_ = false;
			if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_clutch_ = false;
			//flag_drive_mode_ = false;
			//flag_steer_mode_ = false;
			shift_auto_ = false;
			safety_error_message << "target angle over , " << deg;
			//std::cout << safety_error_message.str() << std::endl;
			//system("aplay -D plughw:PCH /home/autoware/one33.wav");
			can_send();
		}

		publishStatus(safety_error_message.str());
		twist_ = *msg;
	}

	void callbackStrokeMode(const std_msgs::Empty::ConstPtr &msg)
	{
		std::cout << "sub StrokeMode" << std::endl;
		drive_control_mode_ = MODE_STROKE;
	}

	void callbackVelocityMode(const std_msgs::Empty::ConstPtr &msg)
	{
		std::cout << "sub VelocityMode" << std::endl;
		drive_control_mode_ = MODE_VELOCITY;
	}

	void callbackDriveControl(const std_msgs::Int8::ConstPtr &msg)
	{
		if(msg->data == autoware_can_msgs::MicroBusCan501::DRIVE_MODE_VELOCITY)
		{
			std::cout << "sub VelocityMode" << std::endl;
			drive_control_mode_ = MODE_VELOCITY;
		}
		else if(msg->data == autoware_can_msgs::MicroBusCan501::DRIVE_MODE_STROKE)
		{
			std::cout << "sub StrokeMode" << std::endl;
			drive_control_mode_ = MODE_STROKE;
		}
		else std::cout << "Control mode flag error" << std::endl;
	}

	void callbackShiftAuto(const std_msgs::Bool::ConstPtr &msg)
	{
		std::string str = (msg->data == true) ? "shift_auto" : "shift_manual";
		std::cout << str << std::endl;
		shift_auto_ = msg->data;
	}

	void callbackShiftPosition(const std_msgs::UInt8::ConstPtr &msg)
	{
		std::cout << "shift position : " << (int)msg->data << std::endl;
		shift_position_ = msg->data;
	}

	void automaticDoorSet(unsigned char flag)
	{
		automatic_door_ = flag;

		ros::Time nowtime = ros::Time::now();
		automatic_door_time_ = ros::Time(nowtime.sec + 5, nowtime.nsec);
	}

	void callbackAutomaticDoor(const std_msgs::UInt8::ConstPtr &msg)
	{
		std::cout << "automatic door : " << (int)msg->data << std::endl;
		automaticDoorSet(msg->data);
	}

	/*void callbackConfigInterface(const autoware_config_msgs::ConfigMicrobusInterface::ConstPtr &msg)
	{
		interface_config_ = *msg;
	}*/

	void callbackConfigMicroBusCan(const autoware_config_msgs::ConfigMicroBusCan::ConstPtr &msg)
	{
		setting_ = *msg;
		/*setting_.velocity_limit = 50;
		setting_.velocity_stop_th = 4.0;
		setting_.pedal_stroke_center = 0;
		setting_.pedal_stroke_max = 850;
		setting_.pedal_stroke_min = -500;
		setting_.brake_stroke_stopping_med = -370;*/
		//setting_.accel_stroke_offset = 10;
		//setting_.brake_stroke_offset = -10;
		setting_.brake_stroke_stopping_med = 0;

		/*setting_.pedal_stroke_max - setting_.pedal_stroke_min
		if(setting_.pedal_stroke_max - setting_.pedal_stroke_min > 1300 ||
				)
			config_result = ERROR_STROKE_MAX_MIN_INCONSISTENCY;
		else if(setting_.pedal_stroke_max - setting_.pedal_stroke_center )*/
		std::string safety_error_message = "";
		publishStatus(safety_error_message);
	}

	void callbackConfigLocalizerSwitch(const autoware_config_msgs::ConfigLocalizerSwitchFusion::ConstPtr &msg)
	{
		config_localizer_switch_ = *msg;
	}

	void callbackWaypointParam(const autoware_msgs::WaypointParam::ConstPtr &msg)
	{
		pedal_ = msg->microbus_pedal;

		if(msg->localizer_check > 0)
		{
			config_localizer_switch_.localizer_check = msg->localizer_check;
		}

		if(msg->automatic_door == 2 && msg->automatic_door != waypoint_param_.automatic_door)
		{
			std::cout << "automatic_door : " << msg->automatic_door << std::endl;
			automaticDoorSet(2);
		}
		else if(msg->automatic_door == 1 && msg->automatic_door != waypoint_param_.automatic_door)
		{
			std::cout << "automatic_door : " << msg->automatic_door << std::endl;
			automaticDoorSet(1);
		}

		if(can_receive_502_.clutch == false && can_receive_503_.clutch == false && blinker_param_sender_ == true)
		{
			blinkerStop();
			blinker_param_sender_ = false;
		}
		else if(msg->blinker == 1 && (can_receive_502_.clutch == true || can_receive_503_.clutch == true))
		{
			blinkerLeft();
			blinker_param_sender_ = true;
		}
		else if(msg->blinker == 2 && (can_receive_502_.clutch == true || can_receive_503_.clutch == true))
		{
			blinkerRight();
			blinker_param_sender_ = true;
		}
		else if(msg->blinker == 0 && (can_receive_502_.clutch == true || can_receive_503_.clutch == true))
		{
			blinkerStop();
			blinker_param_sender_ = false;
		}

		if(msg->liesse.shift >= 0)
		{
			shift_position_ = msg->liesse.shift;
		}

		if(msg->steer_correction > -1000)
		{
			steer_correction_ = msg->steer_correction;
			if(steer_correction_ > 500 || steer_correction_ < -500) steer_correction_ = 0;
		}

		if(msg->accel_stroke_offset >= 0 && msg->accel_stroke_offset <= 300)
		{ 
			setting_.accel_stroke_offset = msg->accel_stroke_offset;
		}

		if(msg->accel_avoidance_distance_min >= 0 && msg->accel_avoidance_distance_min <= 100)
		{
			accel_avoidance_distance_min_ = msg->accel_avoidance_distance_min;
			//std::cout << "kkk accel_avoidance_distance_min:  " << accel_avoidance_distance_min_ << std::endl;
		}
		if(msg->stop_stroke_max >= 300 && msg->stop_stroke_max <=500)
		{
			stop_stroke_max_ = msg->stop_stroke_max;
			//std::cout << "kkk stop_stroke_min : " << stop_stroke_max_ << std::endl;
		}
		if(msg->accel_stroke_max >= 300 && msg->accel_stroke_max <= 850)
		{
			setting_.pedal_stroke_max = msg->accel_stroke_max;
		}
		if(msg->k_accel_p_velocity >= 0 && msg->k_accel_p_velocity <= 2)
			setting_.k_accel_p_velocity = msg->k_accel_p_velocity;
		if(msg->k_accel_i_velocity >= 0 && msg->k_accel_i_velocity <= 2)
			setting_.k_accel_i_velocity = msg->k_accel_i_velocity;
		if(msg->k_accel_d_velocity >= 0 && msg->k_accel_d_velocity <= 2)
			setting_.k_accel_d_velocity = msg->k_accel_d_velocity;
		if(msg->k_brake_p_velocity >= 0 && msg->k_brake_p_velocity <= 2)
			setting_.k_brake_p_velocity = msg->k_brake_p_velocity;
		if(msg->k_brake_i_velocity >= 0 && msg->k_brake_i_velocity <= 2)
			setting_.k_brake_i_velocity = msg->k_brake_i_velocity;
		if(msg->k_brake_d_velocity >= 0 && msg->k_brake_d_velocity <= 2)
			setting_.k_brake_d_velocity = msg->k_brake_d_velocity;

		if(msg->in_accel_mode == 1) in_accel_mode_ = true;
		else in_accel_mode_ = false;
		if(msg->in_brake_mode == 1) in_brake_mode_ = true;
		else in_brake_mode_ = false;

		if(msg->use_stopper_distance == 0) use_stopper_distance_ = false;
		else use_stopper_distance_ = true;
		if(msg->stopper_distance1 > 0) setting_.stopper_distance1 = msg->stopper_distance1;
		if(msg->stopper_distance2 > 0) setting_.stopper_distance2 = msg->stopper_distance2;
		if(msg->stopper_distance3 > 0) setting_.stopper_distance3 = msg->stopper_distance3;
		
		waypoint_param_ = *msg;
	}

	void callbackWaypoints(const autoware_msgs::Lane::ConstPtr &msg)
	{
		if(msg->waypoints.size() < 1)
		{
			waypoint_id_ = -1;
		}

		waypoint_id_ = msg->waypoints[1].waypoint_param.id;
		tf::quaternionMsgToTF(msg->waypoints[1].pose.pose.orientation, waypoint_orientation_);
		double roll, pitch, yaw;
		tf::Matrix3x3 mat(waypoint_orientation_);
		mat.getRPY(roll, pitch, yaw);
		waypoint_angle_ = yaw;
	}

	void callbackPositionChecker(const autoware_msgs::PositionChecker::ConstPtr &msg)
	{
		position_checker_ = *msg;
		if(setting_.use_position_checker == true && use_safety_localizer_ == true)
		{
			if(msg->stop_flag != 0)// && can_receive_502_.clutch == true)
			{
				if(can_receive_501_.drive_auto == autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
					drive_clutch_ = false;
				if(can_receive_501_.steer_auto == autoware_can_msgs::MicroBusCan501::STEER_AUTO)
					steer_clutch_ = false;
				//flag_drive_mode_ = false;
				//flag_steer_mode_ = false;
				shift_auto_ = false;
				std::cout << "Denger! Autoware stop flag : " << msg->stop_flag << std::endl;
				std::stringstream safety_error_message;
				safety_error_message << "positon error : " << msg->stop_flag;
				publishStatus(safety_error_message.str());
				//system("aplay -D plughw:PCH /home/autoware/one33.wav");
				can_send();
			}
			else
			{
				std::stringstream safety_error_message;
				safety_error_message << "";
				publishStatus(safety_error_message.str());
				//can_send();
			}
		}
		else
		{
			std::stringstream safety_error_message;
			safety_error_message << "";
			publishStatus(safety_error_message.str());
			//can_send();
		}
	}

	void publishStatus(std::string safety_error_message)
	{
		autoware_can_msgs::MicroBusCanSenderStatus msg;
		msg.header.stamp = ros::Time::now();
		msg.use_position_checker = setting_.use_position_checker;
		msg.use_input_steer = input_steer_mode_;
		msg.use_input_drive = input_drive_mode_;
		msg.use_velocity_topic = use_velocity_data_;
		msg.position_check_stop = position_checker_.stop_flag;
		msg.angle_limit_over = angle_limit_over_;
		if(safety_error_message != "") msg.safety_error_message = safety_error_message;
		else msg.safety_error_message = "";
		std::cout << msg.safety_error_message << std::endl;
		pub_microbus_can_sender_status_.publish(msg);
	}

	void callbackInputSteerFlag(const std_msgs::Bool::ConstPtr &msg)
	{
		input_steer_mode_ = msg->data;
		std::cout << "aaa" << std::endl;
		std::string safety_error_message = "";
		publishStatus(safety_error_message);
	}

	void callbackInputSteerValue(const std_msgs::Int16::ConstPtr &msg)
	{
		input_steer_ = msg->data;
		std::cout << input_steer_ << std::endl;
	}

	void callbackInputDriveFlag(const std_msgs::Bool::ConstPtr &msg)
	{
		input_drive_mode_ = msg->data;
		std::cout << "ccc" << std::endl;
		std::string safety_error_message = "";
		publishStatus(safety_error_message);
	}

	void callbackInputDriveValue(const std_msgs::Int16::ConstPtr &msg)
	{
		input_drive_ = msg->data;
		std::cout << input_drive_ << std::endl;
	}

	void callbackLightHigh(const std_msgs::Bool::ConstPtr &msg)
	{
		light_high_ = msg->data;
		std::string str = (light_high_) ? "true" : "false";
		std::cout << "light_high : " << str << std::endl;
	}

	void callbackLightLow(const std_msgs::Bool::ConstPtr &msg)
	{
		light_low_ = msg->data;
		std::string str = (light_low_) ? "true" : "false";
		std::cout << "light_low : " << str << std::endl;
	}

	void callbackLightSmall(const std_msgs::Bool::ConstPtr &msg)
	{
		light_small_ = msg->data;
		std::string str = (light_small_) ? "true" : "false";
		std::cout << "light_small : " << str << std::endl;
	}

	void callbackDriveClutch(const std_msgs::Bool::ConstPtr &msg)
	{
		drive_clutch_ = msg->data;
		ros::Time nowtime = ros::Time::now();
		drive_clutch_timer_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void callbackSteerClutch(const std_msgs::Bool::ConstPtr &msg)
	{
		steer_clutch_ = msg->data;
		ros::Time nowtime = ros::Time::now();
		steer_clutch_timer_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void blinkerRight()
	{
		blinker_right_ = true;
		std::cout << "blinker right" << std::endl;
		ros::Time nowtime = ros::Time::now();
		blinker_right_time_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void callbackBlinkerRight(const std_msgs::Bool::ConstPtr &msg)
	{
		blinkerRight();
		blinker_param_sender_ = false;
	}

	void blinkerLeft()
	{
		blinker_left_ = true;
		std::cout << "blinker left" << std::endl;
		ros::Time nowtime = ros::Time::now();
		blinker_left_time_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void callbackBlinkerLeft(const std_msgs::Bool::ConstPtr &msg)
	{
		blinkerLeft();
		blinker_param_sender_ = false;
	}

	void blinkerStop()
	{
		blinker_stop_ = true;
		std::cout << "blinker stop" << std::endl;
		ros::Time nowtime = ros::Time::now();
		blinker_stop_time_ = ros::Time(nowtime.sec + 1, nowtime.nsec);
	}

	void callbackBlinkerStop(const std_msgs::Bool::ConstPtr &msg)
	{
		blinkerStop();
		blinker_param_sender_ = false;
	}

	void bufset_mode(unsigned char *buf)
	{
		unsigned char mode = 0;
		if(flag_drive_mode_ == true) mode |= drive_control_mode_;
		if(flag_steer_mode_ == true) mode |= 0xA0;
		buf[0] = mode;  buf[1] = 0;
	}

	double handle_control_max_speed = 50; //
	double handle_control_min_speed = 10; //
	double handle_control_ratio = 1.0/32.0;
	void bufset_steer(unsigned char *buf)
	{
		short steer_val;
		if(input_steer_mode_ == false)
		{
			double wheel_ang = twist_.ctrl_cmd.steering_angle;
			double zisoku = twist_.ctrl_cmd.linear_velocity * 3.6;
			double ratio = ((handle_control_ratio - 1.0) * (zisoku - handle_control_min_speed))
			        / (handle_control_max_speed - handle_control_min_speed) + 1;
			if(wheel_ang > 0)
			{
				steer_val = wheel_ang * wheelrad_to_steering_can_value_left + steer_correction_;
			}
			else
			{
				steer_val = wheel_ang * wheelrad_to_steering_can_value_right + steer_correction_;
			}
			std::cout << "steer_correction : " << steer_correction_ << std::endl;
		}
		else steer_val = input_steer_;
		//steer_val -= 600;
		//PID
		double wheel_base = 3.935;
//		steer_val += _steer_pid_control(difference_toWaypoint_distance_.base_linkdistance);
//		steer_val += _steer_pid_control(wheel_base * tan(difference_toWaypoint_distance_.baselink_angular) + difference_toWaypoint_distance_.baselink_distance);
		//if(waypoint_param_.steer_pid_on > 0)
				steer_val += _steer_pid_control(difference_toWaypoint_distance_.front_baselink_distance);
		if(can_receive_501_.steer_auto != autoware_can_msgs::MicroBusCan501::STEER_AUTO) steer_val = 0;

		unsigned char *steer_pointer = (unsigned char*)&steer_val;
		buf[2] = steer_pointer[1];  buf[3] = steer_pointer[0];
	}


	double _steer_pid_control(double distance)
	{
//P
		double e = distance;
		//I
		double e_i;
		pid_params.plus_steer_diff_sum_distance(e);

		if(current_velocity_.twist.linear.x < 1.0)
		{
			pid_params.clear_steer_diff_distance();
		}

		double sum_diff = pid_params.get_steer_diff_sum_distance();
		if (sum_diff > setting_.steer_max_i && sum_diff >0)
			e_i = setting_.steer_max_i;
		else if(sum_diff < setting_.steer_max_i && sum_diff <0)
			e_i = -setting_.steer_max_i;
		else
			e_i = sum_diff;

		//D
		double e_d = e - pid_params.get_steer_e_prev_distance();

		double target_steer = setting_.k_steer_p_distance* e +
		       setting_.k_steer_i_distance * e_i +
		       setting_.k_steer_d_distance * e_d;
		if(target_steer > 500) target_steer  = 500;
		if(target_steer < -500) target_steer =-500;

		pid_params.set_steer_e_prev_distance(e);
		return target_steer;
	}





	double _accel_stroke_pid_control(double current_velocity, double cmd_velocity)
	{
		//cmd_velocityとcurrent_velocityの差が小さい場合、stepを小さくする
		double accle_stroke_step = setting_.accel_stroke_step_max;//3;
		double vel_sa = cmd_velocity - current_velocity;
		double accel_stroke_adjust_th = (cmd_velocity + current_velocity)/2.0 * (setting_.accel_stroke_adjust_th/ 100.0);
		if(vel_sa < accel_stroke_adjust_th)
		//if(vel_sa < setting_.accel_stroke_adjust_th)
		{
			//accle_stroke_step -= (setting_.accel_stroke_adjust_th-vel_sa)*setting_.accel_stroke_step_max/setting_.accel_stroke_adjust_th;
			accle_stroke_step -= (setting_.accel_stroke_adjust_th-vel_sa)*setting_.accel_stroke_step_max/accel_stroke_adjust_th;
			if(accle_stroke_step < 0.5) accle_stroke_step = setting_.accel_stroke_step_min;
		}

		//ブレーキからアクセルに変わった場合、Iの積算値をリセット
		double stroke = PEDAL_VOLTAGE_CENTER_ - can_receive_503_.pedal_voltage;
		std::cout << "voltage stroke : " << stroke << std::endl;
		std::cout << "voltage center : " << PEDAL_VOLTAGE_CENTER_ << std::endl;
		std::cout << "voltage        : " << can_receive_503_.pedal_voltage << std::endl;
		std::cout << "brake offset   : " << setting_.brake_stroke_offset << std::endl;

		//std::cout << "if accel : " << stroke << " < " << setting_.brake_stroke_offset << std::endl;
		/*if(stroke < setting_.brake_stroke_offset)
		{
			pid_params.set_accel_e_prev_velocity(0);
			pid_params.set_accel_e_prev_acceleration(0);
			pid_params.set_stop_stroke_prev(0);
			return 0;
		}*/
		if(pid_params.get_stop_stroke_prev() > 0)
		{
			double ret = pid_params.get_stop_stroke_prev();
			ret -= accle_stroke_step;
			if(ret < 0) ret = 0;
			pid_params.set_stop_stroke_prev(ret);
			return -ret;
		}
		//pid_params.set_accel_e_prev_velocity(0);
		//pid_params.set_accel_e_prev_acceleration(0);
		pid_params.set_brake_e_prev_velocity(0);
		pid_params.set_brake_e_prev_acceleration(0);
		pid_params.set_stroke_prev(0);

		//P
		double e = cmd_velocity - current_velocity;
		//std::cout << "if accel : " << current_velocity << "," << cmd_velocity << "," << e << std::endl;
		//std::cout << "accel e : " << e << std::endl;
		//std::cout << "cmd vel : " << cmd_velocity << std::endl;
		//std::cout << "cur vel : " << current_velocity << std::endl;;

		//I
		double e_i;
		pid_params.plus_accel_diff_sum_velocity(e);
		if (pid_params.get_acclel_diff_sum_velocity() > setting_.accel_max_i)
			e_i = setting_.accel_max_i;
		else
			e_i = pid_params.get_acclel_diff_sum_velocity();

		//D
		double e_d = e - pid_params.get_accel_e_prev_velocity();

		double target_accel_stroke = setting_.k_accel_p_velocity * e +
		       setting_.k_accel_i_velocity * e_i +
		       setting_.k_accel_d_velocity * e_d;

		pid_params.set_accel_e_prev_velocity(e);

		double ret = target_accel_stroke;
		if(ret > setting_.pedal_stroke_max)
			ret = setting_.pedal_stroke_max;
		else if (ret < setting_.pedal_stroke_center)
			ret = setting_.pedal_stroke_center;
		if(ret < setting_.accel_stroke_offset) ret = setting_.accel_stroke_offset;
		//if(stopper_distance_ <= 100 && stopper_distance_ >=0 && 
		//	current_velocity >= 10.0 && ret > 0) ret = 0;

		if(pid_params.get_stroke_prev() < 0 && ret >= 0)
		{
			double tmp = pid_params.get_stroke_prev() + accle_stroke_step;
			if(tmp < ret) ret = tmp;
		}
		//ブレーキをゆっくり踏む
		/*if(pid_params.get_stroke_prev() < 0.0 && pid_params.get_stroke_prev() < ret)
		{
			double tmp = pid_params.get_stroke_prev() - accle_stroke_step;
			if(tmp > ret) ret = tmp;
			if(ret < setting_.pedal_stroke_min) ret = setting_.pedal_stroke_min;
		}*/

		send_step_ = accle_stroke_step;
		pid_params.set_stroke_prev(ret);
		return ret;
	}

	double _brake_stroke_pid_control(double current_velocity, double cmd_velocity, double acceleration)
	{
		double brake_stroke_step = setting_.brake_stroke_step_max;//2;
		double vel_sa = current_velocity - cmd_velocity;
		double brake_stroke_adjust_th = (current_velocity + cmd_velocity)/2 * (setting_.brake_stroke_adjust_th / 100.0);
		if(vel_sa < brake_stroke_adjust_th)
		//if(vel_sa < setting_.brake_stroke_adjust_th)
		{
			//brake_stroke_step -= (setting_.brake_stroke_adjust_th-vel_sa)*(setting_.accel_stroke_step_max-1)/setting_.brake_stroke_adjust_th;
			brake_stroke_step -= (setting_.brake_stroke_adjust_th-vel_sa)*(setting_.accel_stroke_step_max-1)/brake_stroke_adjust_th;
			if(brake_stroke_step < 1) brake_stroke_step = setting_.brake_stroke_step_min;
		}
		bool use_step_flag = true;

		if(pid_params.get_stroke_prev() > 0)
		{
			//if(current_velocity > cmd_velocity)
			{
				pid_params.set_stroke_prev(pid_params.get_stroke_prev()-brake_stroke_step);
				return pid_params.get_stroke_prev() - brake_stroke_step;
			}
			//else return pid_params.get_stroke_prev();
		}

		//std::cout << "cur" << current_velocity << "  cmd" << cmd_velocity << std::endl;
		//アクセルからブレーキに変わった場合、Iの積算値をリセット
		double stroke = PEDAL_VOLTAGE_CENTER_ - can_receive_503_.pedal_voltage;
		//std::cout << "stroke " << stroke << std::endl;
		//std::cout << "if : " << stroke << " > " << setting_.accel_stroke_offset << std::endl;;
		if (stroke > 10)
		{
			std::cout << "ACCEL_PEDAL_STROKE_OFFSET_" << std::endl;
			//pid_params.set_brake_e_prev_velocity(0);
			//pid_params.set_brake_e_prev_acceleration(0);
			pid_params.set_accel_e_prev_velocity(0);
			pid_params.set_accel_e_prev_acceleration(0);
			pid_params.set_stroke_prev(0);
			return 0;
		}


		//velocity PID
		//P
		double e = -1 * (cmd_velocity - current_velocity);
		std::cout << "if : " << cmd_velocity << "," << current_velocity << "," << e << std::endl;
		// since this is braking, multiply -1.
		if (e > 0 && e <= 1) { // added @ 2016/Aug/29
			e = 0;
			pid_params.clear_diff_velocity();
		}
		std::cout << "e " << e << std::endl;
		//I
		double e_i;
		pid_params.plus_brake_diff_sum_velocity(e);
		if (pid_params.get_brake_diff_sum_velocity() > setting_.brake_max_i)
			e_i = setting_.brake_max_i;
		else
			e_i = pid_params.get_brake_diff_sum_velocity();

		//D
		double e_d = e - pid_params.get_brake_e_prev_velocity();

		double target_brake_stroke = setting_.k_brake_p_velocity * e +
		        setting_.k_brake_i_velocity * e_i +
		        setting_.k_brake_d_velocity * e_d;
		pid_params.set_brake_e_prev_velocity(e);

	//	static double target_brake_stroke_prev=target_brake_stroke;
/*
		//distance PID
		if(stopper_distance_ >= 0 && acceleration < -0.01)
		{
			double mps = current_velocity / 3.6;
			//double estimated_stopping_distance = (前方車両の速度 * 前方車両の速度 - mps*mps)/(2.0*acceleration);
			double estimated_stopping_distance = (0 * 0 - mps*mps)/(2.0*acceleration);
			//x = current_velocity*current_velocity / (2.0*acc);
			double cmd_distance = stopper_distance_;
			double current_distance = estimated_stopping_distance;
			//P
			double e_dist = -1 * (cmd_distance - current_distance);
			std::cout << "if : " << cmd_distance << "," << current_distance << "," << e_dist << std::endl;
			// since this is braking, multiply -1.
			if (e_dist > 0 && e_dist <= 1) { // added @ 2016/Aug/29
				e_dist = 0;
				pid_params.clear_diff_distance();
			}
			std::cout << "e_dist " << e_dist << std::endl;
			//I
			double e_dist_i;
			pid_params.plus_brake_diff_sum_distance(e_dist);
			if (pid_params.get_brake_diff_sum_distance() > setting_.brake_max_i)
				e_dist_i = setting_.brake_max_i;
			else
				e_dist_i = pid_params.get_brake_diff_sum_distance();
			double val_plus = setting_.k_brake_p_distance * e_dist +
			        setting_.k_brake_i_distance * e_dist_i;
			std::cout << "stopper_distance plus : " << val_plus << "," << stopper_distance_ << "," << estimated_stopping_distance<< std::endl;
	//		target_brake_stroke += val_plus;
    //
			//      if(val_plus < 0 ) target_brake_stroke += val_plus;
			pid_params.set_brake_e_prev_distance(e_dist);
		}
		else
		{
			pid_params.clear_diff_distance();
		}
*/

		//const double stop_stroke = 340.0;
		if(use_stopper_distance_ = true && temporary_fixed_velocity_ <= 0)
		{
			std::cout << "stopper list : "<< setting_.stopper_distance1 << "," << setting_.stopper_distance2 << "," << setting_.stopper_distance3 << std::endl;
			std::cout << "kkk stop_stroke_max : " << stop_stroke_max_ << std::endl;
			if(stopper_distance_ >= setting_.stopper_distance2 && stopper_distance_ <= setting_.stopper_distance1)
			{std::cout << loop_counter_ << " : stopD1" << std::endl;
				/*std::cout << "tbs," << target_brake_stroke;
				double d = stop_stroke - target_brake_stroke;
				if(d < 0) d = 0;
				target_brake_stroke  += d * (1 - stopper_distance_/ 20.0 );
				std::cout << ",tbs," << target_brake_stroke << ",d," << d << ",dis," << stopper_distance_ << std::endl;*/
			}
			else if(stopper_distance_ >= setting_.stopper_distance3 && stopper_distance_ <= setting_.stopper_distance2)
			{std::cout << loop_counter_ << "stopD2" << std::endl;
				/*if(current_velocity > 5.0)
				{
					std::cout << "tbs," << target_brake_stroke;
					double d = stop_stroke - target_brake_stroke;
					if(d < 0) d = 0;
					target_brake_stroke  += d * (1 - stopper_distance_/ 20.0 );
					std::cout << ",tbs," << target_brake_stroke << ",d," << d << ",dis," << stopper_distance_ << std::endl;
				}
				else {
					target_brake_stroke = pid_params.get_stop_stroke_prev();
				}*/
				target_brake_stroke = pid_params.get_stop_stroke_prev();
				if(current_velocity <= 0.5)
				{
					target_brake_stroke -= brake_stroke_step;
					if(target_brake_stroke < 0) target_brake_stroke = 0;
				}
				
			}
			else if(stopper_distance_ >= 0 && stopper_distance_ <= setting_.stopper_distance3)
			{std::cout << loop_counter_ << "stopD3" << std::endl;
				//target_brake_stroke = 0.0 + 500.0 * pow((2.0-distance)/2.0,0.5);
				brake_stroke_step = 0.5;
				target_brake_stroke = 0.0 + stop_stroke_max_ * (2.0 - stopper_distance_)/2.0;
				if(target_brake_stroke < pid_params.get_stop_stroke_prev())
					target_brake_stroke = pid_params.get_stop_stroke_prev();
			}
		}
/*
		if(stopper_distance_ >= 0.5 && stopper_distance_ <= 3 && current_velocity >= 0.1 && target_brake_stroke < 330)
		{
			double d = 400 - target_brake_stroke;
			if(d < 0) d = 0;
 			target_brake_stroke  += d * (1 - stopper_distance_/ 20.0 );
		}
		if(stopper_distance_ >= 0 && stopper_distance_ <= 0.5 && current_velocity < 0.1 && target_brake_stroke > 330)
		{
			target_brake_stroke = 500;
		}
*/
/*
		if(stopper_distance_ <= 30 && stopper_distance_ >0)
		{
			if(current_velocity > 5.0 && fabs(jurk2_twist_) < 10)
			{
				std::cout << "tbs," << target_brake_stroke;
				double d = 500 - target_brake_stroke;
				target_brake_stroke  += d * (1 - stopper_distance_/ 30.0 );
				std::cout << ",tbs," << target_brake_stroke << ",d," << d << ",dis," << stopper_distance_ << std::endl;
				pid_params.set_stop_stroke_prev(target_brake_stroke);// .set_stroke_prev(target_brake_stroke);
			}
			else if(stopper_distance_ >= 2  && fabs(jurk2_twist_) < 10)
			{
				target_brake_stroke = pid_params.get_stop_stroke_prev();
			}
			else if(stopper_distance_ >= 0 && stopper_distance_ <= 2)
			{
						brake_stroke_step = 0.5;
						target_brake_stroke = 0.0 + 500.0 * (2.0 - stopper_distance_)/2.0;
			}
		}
		*/
/*
		bool lessthan2 = false;
		static double target_brake_stroke_old=0;
		if(stopper_distance_ <= 30 && stopper_distance_ >0)
		{		
				double d = ( 300 > target_brake_stroke) ? 300 - target_brake_stroke:300;
				target_brake_stroke  += d * (1 - stopper_distance_/ 30.0 );
		}
*/
/*
		bool lessthan2 = false;
		static double target_brake_stroke_old=0;
		if(stopper_distance_ <= 30 && stopper_distance_ >0)
		{
			if(current_velocity > 5.0 && fabs(jurk2_twist_) < 10)
			{
				double d = 300 - target_brake_stroke;
				target_brake_stroke  += d * (1 - stopper_distance_/ 30.0 );
			}
			else if(stopper_distance_ >= 2 && fabs(jurk2_twist_) < 10)
			{
				target_brake_stroke = pid_params.get_stop_stroke_prev();
				double mps = current_velocity /3.6;
				//double estimated_stopping_distance = (前方車両の速度 * 前方車両の速度 - mps*mps)/(2.0*acceleration);
				double estimated_stopping_distance = (0 * 0 - mps*mps)/(2.0*acceleration2_twist_);
				std_msgs::Float64 esd;
				esd.data = estimated_stopping_distance;
				pub_estimate_stopper_distance_.publish(esd);
				target_brake_stroke = pid_params.get_stop_stroke_prev();
				double P_d = -13;
				double e_d = stopper_distance_ - estimated_stopping_distance; 
			//	target_brake_stroke += e_d * P_d;
			}
			else if(stopper_distance_ >= 0 && stopper_distance_ < 2 )
			{
					if(!lessthan2)
					{
						target_brake_stroke_old = pid_params.get_stop_stroke_prev();
						lessthan2 = true;
					}
					brake_stroke_step = 0.5;
					//double d = (300 > target_brake_stroke_old) ? 300- target_brake_stroke_old:300;
					//target_brake_stroke = target_brake_stroke_old +  d * (2.0 - stopper_distance_)/2.0;
					target_brake_stroke = target_brake_stroke_old +  500 * (2.0 - stopper_distance_)/2.0;
			}
		}
*/


/*
		if(stopper_distance_ <= 40 && stopper_distance_ >0)
		{
			if(current_velocity > 5.0 && fabs(jurk2_twist_) < 10)
			{
				std::cout << "tbs," << target_brake_stroke;
				double d = 300 - target_brake_stroke;
				target_brake_stroke  += d * (1 - stopper_distance_/ 30.0 );
				std::cout << ",tbs," << target_brake_stroke << ",d," << d << ",dis," << stopper_distance_ << std::endl;
				pid_params.set_stop_stroke_prev(target_brake_stroke);// .set_stroke_prev(target_brake_stroke);
			}
			else if(stopper_distance_ >= 2  && fabs(jurk2_twist_) < 10)
			{
				double mps = current_velocity / 3.6;
				//double estimated_stopping_distance = (前方車両の速度 * 前方車両の速度 - mps*mps)/(2.0*acceleration);
				double estimated_stopping_distance = (0 * 0 - mps*mps)/(2.0*acceleration);
				target_brake_stroke = pid_params.get_stop_stroke_prev();
			//	double P_d = -13;
			//double e_d = stopper_distance_ - estimated_stopping_distance; 
				//target_brake_stroke += e_d * P_d;
			}
			else if(stopper_distance_ >= 1 && stopper_distance_ <= 2)
			{
					if(current_velocity > 0.25 )
					{
						target_brake_stroke = pid_params.get_stop_stroke_prev();
					//	double cmd_acceralation = - current_velocity * current_velocity / (2 * stopper_distance_);				
				//		double e_a = cmd_acceralation - acceleration2_twist_ ;
				//		double P_a = -1;
				//		target_brake_stroke += e_a * P_a;
						brake_stroke_step = 0.5;
						if(fabs(jurk2_twist_) < 10) 
						{
							double d = 500 - target_brake_stroke;
							target_brake_stroke = target_brake_stroke + d * (2.0 - stopper_distance_)/2.0;
						}
					}
					else if(current_velocity < 0.25 )
					{
						brake_stroke_step = 0.5;
						target_brake_stroke = 0.0 + 500.0 * (2.0 - stopper_distance_)/2.0;
					}
			}
			else if(stopper_distance_ >= 0 && stopper_distance_ <= 1)
			{
				//target_brake_stroke = 0.0 + 500.0 * pow((2.0-distance)/2.0,0.5);
				//brake_stroke_step = 2;
				use_step_flag = false;
				target_brake_stroke = 0.0 + 500 * (1.0 - stopper_distance_)/1.0;
			}
		}
*/
/*
		if(stopper_distance_ <= 25 && stopper_distance_ > 0)
		{
			double mps = current_velocity / 3.6;
			//double estimated_stopping_distance = (前方車両の速度 * 前方車両の速度 - mps*mps)/(2.0*acceleration);
			double estimated_stopping_distance = (0 * 0 - mps*mps)/(2.0*acceleration);
			std_msgs::Float64 esd;
			esd.data = estimated_stopping_distance;
			pub_estimate_stopper_distance_.publish(esd);
			if(current_velocity > 15.0 || abs(jurk2_twist_) < 10 )
			//if(estimated_stopping_distance > stopper_distance_)// && jurk2_twist_ < 4000 )
			{
				std::cout << "tbs," << target_brake_stroke;
				double d = 500 - target_brake_stroke;
				target_brake_stroke  += d * (1 - stopper_distance_/ 25.0 );
				std::cout << ",tbs," << target_brake_stroke << ",d," << d << ",dis," << stopper_distance_ << std::endl;
				//pid_params.set_stop_stroke_prev(target_brake_stroke);
			}
			else if(stopper_distance_ >= 1)
			{
				brake_stroke_step = 0.5;
				target_brake_stroke = pid_params.get_stop_stroke_prev() - 70;
				//PID distance
				double P_d = -13;
				double e = stopper_distance_ - estimated_stopping_distance; 
				target_brake_stroke += e * P_d;
				//target_brake_stroke += -5*fabs((fabs(jurk2_twist_) - 10));
				if(current_velocity < 0.5  || fabs(jurk2_twist_) > 10)
				{
					target_brake_stroke -= 70;
				}
			}
			else if(stopper_distance_ >= 0 && stopper_distance_ <= 1)
			{
				brake_stroke_step = 0.1;
				//target_brake_stroke = 0.0 + 500.0 * pow((2.0-distance)/2.0,0.5);
				target_brake_stroke = pid_params.get_stop_stroke_prev();
				std::cout <<"get_stroke_prev" << target_brake_stroke << std::endl;
				double d = 500 - target_brake_stroke;
				target_brake_stroke = target_brake_stroke + d * (1.0 - stopper_distance_)/1.0;
				//pid_params.set_stop_stroke_prev(target_brake_stroke);
			}
		}
*/


	//	target_brake_stroke_prev = target_brake_stroke;

		double ret = target_brake_stroke;//std::cout << "ret " << setting_.k_brake_p_velocity << std::endl;
		if (-ret < setting_.pedal_stroke_min)
			ret = -setting_.pedal_stroke_min;
		else if (-ret > setting_.pedal_stroke_center)
			ret = -setting_.pedal_stroke_center;

		if(use_step_flag == true)
		{
			/*if(pid_params.get_stop_stroke_prev() < 0 && ret >= 0)
			{
				double tmp = pid_params.get_stop_stroke_prev() + brake_stroke_step;
				if(tmp < ret) ret = tmp;
			}*/
			
			if(pid_params.get_stop_stroke_prev()-ret >= 50 && pid_params.get_stop_stroke_prev() >= 0)
			{
				ret = pid_params.get_stop_stroke_prev();
				ret -= brake_stroke_step;
				if(ret < 0) ret = 0;
				//std::cout << "brake_ret 1" << std::endl;
			}
			//ブレーキをゆっくり踏む
			else if(pid_params.get_stop_stroke_prev() > 0.0 && pid_params.get_stop_stroke_prev() < ret)
			{
				double tmp = pid_params.get_stop_stroke_prev() - brake_stroke_step;
				if(tmp > ret) ret = tmp;
				if(-ret < setting_.pedal_stroke_min) ret = -setting_.pedal_stroke_min;
				//std::cout << "brake_ret 2" << std::endl;
			}
			else std::cout << "brake_ret 3" << std::endl;
		}

		//if(ret < 0) ret = 0;
		//if(ret > -setting_.pedal_stroke_min) ret = -setting_.pedal_stroke_min;
		send_step_ = brake_stroke_step;
		pid_params.set_stop_stroke_prev(ret);
		return -ret;
	}

	short _stopping_control(double current_velocity)
	{std::cout << "stop cur : " << current_velocity << std::endl;
		if (current_velocity < (VELOCITY_ZERO_VALUE_+20)/100.0)
		{
			int gain = (int)(((double)setting_.pedal_stroke_min)*can_receive_502_.cycle_time);
			std::cout << "stop  gain : " << gain << std::endl;
			std::cout << "cycle time : " << can_receive_502_.cycle_time << std::endl;
			std::cout << "stroke min : " << setting_.pedal_stroke_min << std::endl;
			double ret = pid_params.get_stop_stroke_prev() + gain;
			if((int)ret > setting_.pedal_stroke_min) ret = setting_.pedal_stroke_min;
			if((int)ret < setting_.brake_stroke_stopping_med) ret = setting_.brake_stroke_stopping_med;
			return ret;
		}
		else
		{
			return setting_.brake_stroke_stopping_med;
		}
	}

	double _keep_control()
	{
		if(routine_.data == "accel")
		{
			pid_params.set_stop_stroke_prev(0);
			return pid_params.get_stroke_prev();
		}
		else if(routine_.data == "brake")
		{
			pid_params.set_stroke_prev(0);
			return -pid_params.get_stop_stroke_prev();
		}
		else if(routine_.data == "keep")
		{
			if(pid_params.get_stroke_prev() > 0)
			{
				double accel = pid_params.get_stroke_prev();
				accel -= 2;
				if(accel < 0) accel = 0;
				pid_params.set_stroke_prev(accel);
				return accel;
			}
			else
			{
				double brake = pid_params.get_stop_stroke_prev();
				brake -= 2;
				if(brake < 0) brake = 0;
				pid_params.set_stop_stroke_prev(brake);
				return -brake;
			}
		}
		return 0;
	}

	void bufset_drive(unsigned char *buf, double current_velocity, double acceleration, double stroke_speed)
	{
		ros::Time nowtime = ros::Time::now();
		if(can_receive_501_.drive_mode == autoware_can_msgs::MicroBusCan501::DRIVE_MODE_VELOCITY)
		{
			/*short drive_val;
			if(input_drive_mode_ == false)
			{
				std::cout <<"jjj : " << twist_.ctrl_cmd.linear_velocity;
				double linearx = twist_.ctrl_cmd.linear_velocity;
				double twist_drv = linearx *3.6 * 100;
				drive_val = twist_drv;
			}
			else drive_val = input_drive_;
			if(can_receive_501_.drive_auto != autoware_can_msgs::MicroBusCan501::DRIVE_AUTO)
				drive_val = 0;
			if(can_receive_503_.clutch == false)
			{
				drive_val = can_receive_502_.velocity_actual;
				shift_auto_ = false;
			}
			unsigned char *drive_point = (unsigned char*)&drive_val;
			buf[4] = drive_point[1];  buf[5] = drive_point[0];*/
		}
		else
		{
			/*double cmd_velocity;
			if(input_drive_mode_ == false)
				cmd_velocity = twist_.ctrl_cmd.linear_velocity * 3.6;
			else
				cmd_velocity = input_drive_ / 100.0;*/
			double cmd_velocity = twist_.ctrl_cmd.linear_velocity * 3.6;
			if(input_drive_mode_ == true && can_receive_501_.drive_auto)
				cmd_velocity = input_drive_ / 100.0;
			if(config_current_velocity_conversion_.enable == true && can_receive_502_.clutch == false && can_receive_503_.clutch == true)
				cmd_velocity = cruse_velocity_;

			//std::cout << "cur_cmd : " << current_velocity << "," << cmd_velocity << std::endl;
			double cv_s = current_velocity /3.6;
			if(acceleration <= 0 && stopper_distance_ >= 0)
				std::cout << "teisi," << - cv_s*cv_s/(2.0*acceleration) << "," << stopper_distance_ <<  std::endl;

			std::cout << "auto_mode" << std::endl;
			double new_stroke = 0;
			std::cout << "cur_cmd : " << current_velocity << "," << cmd_velocity << "," << setting_.velocity_limit << std::endl;
			//std::cout << "if : " << cmd_velocity << " > " << current_velocity << std::endl;
			//std::cout << "if : " << cmd_velocity << " > " "0.0" << std::endl;
			//std::cout << "if : " <<current_velocity << " > " << setting_.velocity_limit << std::endl;
			//加速判定
			std::cout << "kkk accel_avoidance_distance_min : " << accel_avoidance_distance_min_ << std::endl;
			double accel_mode_avoidance_distance = (current_velocity > accel_avoidance_distance_min_) ? current_velocity : accel_avoidance_distance_min_;
			if (checkMobileyeObstacleStop(nowtime) == false
					&& fabs(cmd_velocity) > current_velocity + setting_.acceptable_velocity_variation
			        && current_velocity < setting_.velocity_limit
			        && (stopper_distance_<0 || stopper_distance_>accel_mode_avoidance_distance)
					&& in_accel_mode_ == true)
			{
				std::cout << " stroke drive" << std::endl;
				pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_ACCEL_);
			}
			//減速判定
			else if(fabs(cmd_velocity) < current_velocity - setting_.acceptable_velocity_variation
			         && fabs(cmd_velocity) > 0.0 || (stopper_distance_>=0 && stopper_distance_ <=current_velocity)
					 && in_brake_mode_ == true)
			{
				std::cout << "stroke brake" << std::endl;
				pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_BRAKE_);
			}
			//停止線判定
			else if (stopper_distance_ >= 0 && stopper_distance_ < accel_avoidance_distance_min_ && in_brake_mode_ == true)
			{
				std::cout << "stroke distance" << std::endl;
				pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_BRAKE_);
			}
			//停止判定
			/*else if(cmd_velocity == 0.0 && current_velocity > 0.0)//VELOCITY_ZERO_VALUE_/100.0)
			{std::cout << "stroke stop" << std::endl;
				if(current_velocity < setting_.velocity_stop_th)
				{
					//new_stroke = _stopping_control(current_velocity);
					//pid_params.set_stop_stroke_prev(new_stroke);
					pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_STOP_);
				}
				else
				{
					//new_stroke = _brake_stroke_pid_control(current_velocity, cmd_velocity);
					pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_BRAKE_);
				}
			}*/
			else if(current_velocity > setting_.velocity_limit)
			{
				std::cout << "stroke over limit" << std::endl;
				pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_KEEP_);
			}
			else {
				pid_params.set_stroke_state_mode_(PID_params::STROKE_STATE_MODE_KEEP_);
			}

			//td_msgs::String routine;
			switch(pid_params.get_stroke_state_mode_())
			{
			case PID_params::STROKE_STATE_MODE_ACCEL_:
				new_stroke = _accel_stroke_pid_control(current_velocity, cmd_velocity);//, &stroke_speed);
				routine_.data = "accel";
				pub_stroke_routine_.publish(routine_);
				break;
			case PID_params::STROKE_STATE_MODE_BRAKE_:
				new_stroke = _brake_stroke_pid_control(current_velocity, cmd_velocity, acceleration);//, &stroke_speed);
				/*if(stopper_distance_ >= 0 && stopper_distance_ <= 1.5 &&
					new_stroke > pid_params.get_stroke_prev())
						new_stroke = pid_params.get_stroke_prev();*/
				routine_.data = "brake";
				pub_stroke_routine_.publish(routine_);
				break;
			case PID_params::STROKE_STATE_MODE_STOP_:
				new_stroke = _stopping_control(current_velocity);
				pid_params.set_stop_stroke_prev(new_stroke);
				break;
			case PID_params::STROKE_STATE_MODE_KEEP_:
				new_stroke = _keep_control();//pid_params.get_stroke_prev();
				routine_.data = "keep";
				pub_stroke_routine_.publish(routine_);
				break;
			}

			//ブレーキを離す場合、徐々に離す
			/*if(pid_params.get_stroke_prev() < 0 && new_stroke >= 0)
			{
				double tmp = pid_params.get_stroke_prev() + stroke_speed;
				if(tmp < new_stroke) new_stroke = tmp;
			}
			//ブレーキをゆっくり踏む
			if(pid_params.get_stroke_prev() < 0.0 && pid_params.get_stroke_prev() < new_stroke)
			{
				double tmp = pid_params.get_stroke_prev() - stroke_speed;
				if(tmp > new_stroke) new_stroke = tmp;
				if(new_stroke < setting_.pedal_stroke_min) new_stroke = setting_.pedal_stroke_min;
			}
			pid_params.set_stroke_prev(new_stroke);*/

			//AUTOモードじゃない場合、stroke値0をcanに送る
			if(can_receive_501_.drive_auto != autoware_can_msgs::MicroBusCan501::DRIVE_AUTO ||
			    can_receive_503_.clutch == false)
			{
				pid_params.clear_diff_velocity();
				pid_params.clear_diff_acceleration();
				pid_params.clear_diff_distance();
				pid_params.set_stop_stroke_prev(0);
				pid_params.set_stroke_prev(0);
				short drive_val = 0;
				unsigned char *drive_point = (unsigned char*)&drive_val;
				buf[4] = drive_point[1];  buf[5] = drive_point[0];
				std::cout << "manual_mode" << std::endl;
				return;
			}

			short input_stroke = (short)new_stroke;
			if(input_drive_mode_ == true) input_stroke = input_drive_;
			unsigned char *drive_point = (unsigned char*)&input_stroke;
			buf[4] = drive_point[1];  buf[5] = drive_point[0];
		}
	}

	void bufset_car_control(unsigned char *buf, double current_velocity)
	{
		buf[6] = buf[7] = 0;

		if(emergency_stop_ == 0x2) {buf[6] |= 0x80;  emergency_stop_ = 0;}
		else if(emergency_stop_ == 0x1) {buf[6] |= 0x40;  emergency_stop_ = 0;}
		if(drive_clutch_ == false)
		{
			buf[6] |= 0x20;
			ros::Time time = ros::Time::now();
			if(time > drive_clutch_timer_) drive_clutch_ = true;
			else drive_clutch_ = false;
		}
		else if(interface_lock_ == true) buf[6] |= 0x20;
		//if(use_safety_localizer_ == false) buf[6] |= 0x10;
		//else
		if(steer_clutch_ == false)
		{
			buf[6] |= 0x10;
			ros::Time time = ros::Time::now();
			if(time > steer_clutch_timer_) steer_clutch_ = true;
			else steer_clutch_ = false;
		}
		else if(interface_lock_ == true) buf[6] |= 0x10;
		if(automatic_door_ != 0x0)
		{
			if(automatic_door_ == 0x2) {buf[6] |= 0x08;}
			else if(automatic_door_ == 0x1) {buf[6] |= 0x04;}
			ros::Time time = ros::Time::now();
			if(time > automatic_door_time_)  automatic_door_ = 0x0;
		}
		if(blinker_right_ == true)
		{
			buf[6] |= 0x02; //blinker_right_ = false;
			ros::Time time = ros::Time::now();
			if(time > blinker_right_time_)  blinker_right_ = false;
		}
		else if(blinker_left_ == true)
		{
			buf[6] |= 0x01;
			ros::Time time = ros::Time::now();
			if(time > blinker_left_time_)  blinker_left_ = false;
		}
		else if(blinker_stop_ == true)
		{
			buf[6] |= 0x03;
			ros::Time time = ros::Time::now();
			if(time > blinker_stop_time_)  blinker_stop_ = false;
		}
		if(light_high_ == true) buf[7] |= 0x20;

		if (shift_auto_ == true)
		{
			buf[7] |= 0x08;
			switch (shift_position_)
			{
			case SHIFT_P:
				if(current_velocity > 0) buf[7] |= 0x00;
				break;
			case SHIFT_R:
				if(current_velocity > 0) buf[7] |= 0x01;
				break;
			case SHIFT_N:
				buf[7] |= 0x02;
				break;
			case SHIFT_D:
				buf[7] |= 0x03;
				break;
			case SHIFT_4:
				buf[7] |= 0x04;
				break;
			case SHIFT_L:
				buf[7] |= 0x05;
				break;
			}
		}
	}
public:
	kvaser_can_sender(ros::NodeHandle nh, ros::NodeHandle p_nh)
	    : nh_(nh)
	    , private_nh_(p_nh)
	    , flag_drive_mode_(false)
	    , flag_steer_mode_(false)
	    , input_drive_mode_(true)
	    , input_steer_mode_(true)
	    , input_steer_(0)
	    , input_drive_(0)
	    , drive_control_mode_(MODE_STROKE)
	    , pedal_(0)
	    , shift_auto_(false)
	    , shift_position_(0)
	    , emergency_stop_(false)
	    , light_high_(false)
	    , light_low_(false)
	    , light_small_(false)
	    , blinker_right_(false)
	    , blinker_left_(false)
	    , blinker_stop_(false)
		, blinker_param_sender_(false)
	    , automatic_door_(0)
	    , drive_clutch_(true)
	    , steer_clutch_(true)
	    , config_result(CONFIG_OK)
	    , use_velocity_data_(USE_VELOCITY_TWIST)
	    , use_acceleration_data_(USE_ACCELERATION_IMU)
	    , stopper_distance_(-1)
	    , acceleration1_twist_(0)
	    , acceleration2_twist_(0)
	    , jurk1_twist_(0)
	    , jurk2_twist_(0)
	    , angle_limit_over_(false)
	    , steer_correction_(0)
		, localizer_select_num_(1)
		, accel_avoidance_distance_min_(30)
		, stop_stroke_max_(340)
		, in_accel_mode_(true)
		, in_brake_mode_(true)
		, use_stopper_distance_(true)
		, interface_lock_(false)
		, loop_counter_(0)
		, ndt_warning_count_(0)
		, use_safety_localizer_(true)
		, cruse_velocity_(0)
		, temporary_fixed_velocity_(0)
	{
		/*setting_.use_position_checker = true;
		setting_.velocity_limit = 50;
		setting_.velocity_stop_th = 4.0;
		setting_.accel_max_i = 3000.0;
		setting_.brake_max_i = 500.0;
		setting_.k_accel_p_velocity_list_ = 120.0;
		setting_.k_accel_i_velocity_list_ = 0.1;
		setting_.k_accel_d_velocity_list_ = 0.1;
		setting_.k_brake_p_velocity_list_ = 40.0;
		setting_.k_brake_i_velocity_list_ = 10.0;
		setting_.k_brake_d_velocity_list_ = 10.0;
		setting_.pedal_stroke_center = 0;
		setting_.pedal_stroke_max = 850;
		setting_.pedal_stroke_min = -500;
		setting_.brake_stroke_stopping_med = -400;
		setting_.accel_stroke_offset = 10;
		setting_.accel_stroke_offset = -10;*/

		position_checker_.stop_flag = false;

		int kvaser_channel;
		private_nh_.param<int>("kvaser_channel", kvaser_channel, 0);
		private_nh_.param<int>("use_velocity_data", use_velocity_data_, USE_VELOCITY_TWIST);
		private_nh_.param<int>("use_acceleration_data", use_acceleration_data_, USE_ACCELERATION_IMU);

		can_receive_501_.emergency = true;
		can_receive_501_.blinker_right = can_receive_501_.blinker_left = false;
		canStatus res = kc.init(kvaser_channel, canBITRATE_500K);
		if(res != canStatus::canOK) {std::cout << "open error" << std::endl;}

		lafvc_.add(limit10); lafvc_.add(limit15); lafvc_.add(limit20); lafvc_.add(limit30); lafvc_.add(limit40);

		pub_microbus_can_sender_status_ = nh_.advertise<autoware_can_msgs::MicroBusCanSenderStatus>("/microbus/can_sender_status", 1, true);
		pub_acceleration_write_ = nh_.advertise<std_msgs::String>("/microbus/acceleration_write", 1);
		pub_estimate_stopper_distance_ = nh_.advertise<std_msgs::Float64>("/microbus/estimate_stopper_distance", 1);
		pub_localizer_match_stat_ = nh_.advertise<autoware_msgs::LocalizerMatchStat>("/microbus/localizer_match_stat", 1);
		pub_stroke_routine_ = nh_.advertise<std_msgs::String>("/microbus/stroke_routine", 1);
		pub_vehicle_status_ = nh_.advertise<autoware_msgs::VehicleStatus>("/microbus/vehicle_status", 1);
		pub_velocity_param_ = nh_.advertise<autoware_can_msgs::MicroBusCanVelocityParam>("/microbus/velocity_param", 1);

		sub_microbus_drive_mode_ = nh_.subscribe("/microbus/drive_mode_send", 10, &kvaser_can_sender::callbackDModeSend, this);
		sub_microbus_steer_mode_ = nh_.subscribe("/microbus/steer_mode_send", 10, &kvaser_can_sender::callbackSModeSend, this);
		sub_twist_cmd_ = nh_.subscribe("/vehicle_cmd", 10, &kvaser_can_sender::callbackTwistCmd, this);
		sub_microbus_can_501_ = nh_.subscribe("/microbus/can_receive501", 10, &kvaser_can_sender::callbackMicrobusCan501, this);
		sub_microbus_can_502_ = nh_.subscribe("/microbus/can_receive502", 10, &kvaser_can_sender::callbackMicrobusCan502, this);
		sub_microbus_can_503_ = nh_.subscribe("/microbus/can_receive503", 10, &kvaser_can_sender::callbackMicrobusCan503, this);
		//sub_current_pose_ = nh_.subscribe("/current_pose", 10, &kvaser_can_sender::callbackCurrentPose, this);
		//sub_current_velocity_ = nh_.subscribe("/current_velocity", 10, &kvaser_can_sender::callbackCurrentVelocity, this);
		sub_emergency_reset_ = nh_.subscribe("/microbus/emergency_reset", 10, &kvaser_can_sender::callbackEmergencyReset, this);
		sub_input_steer_flag_ = nh_.subscribe("/microbus/input_steer_flag", 10, &kvaser_can_sender::callbackInputSteerFlag, this);
		sub_input_steer_value_ = nh_.subscribe("/microbus/input_steer_value", 10, &kvaser_can_sender::callbackInputSteerValue, this);
		sub_input_drive_flag_ = nh_.subscribe("/microbus/input_drive_flag", 10, &kvaser_can_sender::callbackInputDriveFlag, this);
		sub_input_drive_value_ = nh_.subscribe("/microbus/input_drive_value", 10, &kvaser_can_sender::callbackInputDriveValue, this);
		sub_stroke_mode_ = nh_.subscribe("/microbus/set_stroke_mode", 10, &kvaser_can_sender::callbackStrokeMode, this);
		sub_velocity_mode_ = nh_.subscribe("/microbus/set_velocity_mode", 10, &kvaser_can_sender::callbackVelocityMode, this);
		sub_drive_control_ = nh_.subscribe("/microbus/drive_control_", 10, &kvaser_can_sender::callbackDriveControl, this);
		sub_waypoint_param_ = nh_.subscribe("/waypoint_param", 10, &kvaser_can_sender::callbackWaypointParam, this);
		//sub_waypoints_ = nh_.subscribe("/waypoint_param", 10, &kvaser_can_sender::callbackWaypointParam, this);
		sub_position_checker_ = nh_.subscribe("/final_waypoints", 10, &kvaser_can_sender::callbackWaypoints, this);
		sub_config_microbus_can_ = nh_.subscribe("/config/microbus_can", 10, &kvaser_can_sender::callbackConfigMicroBusCan, this);
		sub_config_localizer_switch_ = nh_.subscribe("/config/localizer_switch", 10, &kvaser_can_sender::callbackConfigLocalizerSwitch, this);
		sub_shift_auto_ = nh_.subscribe("/microbus/shift_auto", 10, &kvaser_can_sender::callbackShiftAuto, this);
		sub_shift_position_ = nh_.subscribe("/microbus/shift_position", 10, &kvaser_can_sender::callbackShiftPosition, this);
		sub_emergency_stop_ = nh_.subscribe("/microbus/emergency_stop", 10, &kvaser_can_sender::callbackEmergencyStop, this);
		sub_light_high_ = nh_.subscribe("/microbus/light_high", 10, &kvaser_can_sender::callbackLightHigh, this);
		sub_light_low_ = nh_.subscribe("/microbus/light_low", 10, &kvaser_can_sender::callbackLightLow, this);
		sub_light_small_ = nh_.subscribe("/microbus/light_small", 10, &kvaser_can_sender::callbackLightSmall, this);
		sub_blinker_right_ = nh_.subscribe("/microbus/blinker_right", 10, &kvaser_can_sender::callbackBlinkerRight, this);
		sub_blinker_left_ = nh_.subscribe("/microbus/blinker_left", 10, &kvaser_can_sender::callbackBlinkerLeft, this);
		sub_blinker_stop_ = nh_.subscribe("/microbus/blinker_stop", 10, &kvaser_can_sender::callbackBlinkerStop, this);
		sub_automatic_door_ = nh_.subscribe("/microbus/automatic_door", 10, &kvaser_can_sender::callbackAutomaticDoor, this);
		sub_drive_clutch_ = nh_.subscribe("/microbus/drive_clutch", 10, &kvaser_can_sender::callbackDriveClutch, this);
		sub_steer_clutch_ = nh_.subscribe("/microbus/steer_clutch", 10, &kvaser_can_sender::callbackSteerClutch, this);
		sub_econtrol_ = nh_.subscribe("/econtrol", 10, &kvaser_can_sender::callbackEControl, this);
		sub_obtracle_waypoint_ = nh_.subscribe("/obstacle_waypoint", 10, &kvaser_can_sender::callbackObstracleWaypoint, this);
		sub_stopper_distance_ = nh_.subscribe("/stopper_distance", 10, &kvaser_can_sender::callbackStopperDistance, this);
		sub_lidar_detector_objects_ = nh_.subscribe("/detection/lidar_detector/objects", 10, &kvaser_can_sender::callbackLidarDetectorObjects, this);
		sub_imu_ = nh_.subscribe("/imu_raw_tidy", 10, &kvaser_can_sender::callbackImu, this);
		sub_gnss_standard_deviation_ = nh_.subscribe("/gnss_standard_deviation", 10, &kvaser_can_sender::callbackGnssStandardDeviation, this);
		sub_ndt_stat_string = nh_.subscribe("/ndt_monitor/ndt_status", 10, &kvaser_can_sender::callbackNdtStatString, this);
		sub_ndt_stat_ = nh_.subscribe("/ndt_stat", 10, &kvaser_can_sender::callbackNdtStat, this);
		sub_ndt_reliability_ = nh_.subscribe("/ndt_reliability", 10, &kvaser_can_sender::callbackNdtReliability, this);
		sub_gnss_stat_ = nh_.subscribe("/gnss_stat", 10, &kvaser_can_sender::callbackGnssStat, this);
		sub_ndt_pose_ = nh_.subscribe("/ndt_pose", 10, &kvaser_can_sender::callbackNdtPose, this);
		sub_gnss_pose_ = nh_.subscribe("/RTK_gnss_pose", 10, &kvaser_can_sender::callbackGnssPose, this);
		sub_ekf_pose_ = nh_.subscribe("/ekf_pose", 10, &kvaser_can_sender::callbackEkfPose, this);
		sub_ekf_covariance_ = nh_.subscribe("/ekf_pose_with_covariance", 10, &kvaser_can_sender::callbackEkfCovariance, this);
		sub_difference_to_waypoint_distance_ = nh_.subscribe("/difference_to_waypoint_distance", 10, &kvaser_can_sender::callbackDifferenceToWaypointDistance, this);
		sub_difference_to_waypoint_distance_ndt_ = nh_.subscribe("/difference_to_waypoint_distance_ndt", 10, &kvaser_can_sender::callbackDifferenceToWaypointDistanceNdt, this);
		sub_difference_to_waypoint_distance_gnss_ = nh_.subscribe("/difference_to_waypoint_distance_gnss", 10, &kvaser_can_sender::callbackDifferenceToWaypointDistanceGnss, this);
		sub_difference_to_waypoint_distance_ekf_ = nh_.subscribe("/difference_to_waypoint_distance_ekf", 10, &kvaser_can_sender::callbackDifferenceToWaypointDistanceEkf, this);
		sub_localizer_select_num_ = nh_.subscribe("/localizer_select_num", 10, &kvaser_can_sender::callbackLocalizerSelectNum, this);                
		sub_interface_lock_ = nh_.subscribe("/microbus/interface_lock", 10, &kvaser_can_sender::callbackInterfaceLock, this);
		sub_use_safety_localizer_ = nh_.subscribe("/microbus/use_safety_localizer", 10, &kvaser_can_sender::callbackUseSafetyLocalizer, this);
		sub_config_current_velocity_conversion_ = nh_.subscribe("/config/current_velocity_conversion", 10, &kvaser_can_sender::callbackCurrentVelocityConversion, this);
		sub_cruse_velocity_ = nh_.subscribe("/cruse_velocity", 10, &kvaser_can_sender::callbackCruseVelocity, this);
		sub_mobileye_frame_ = nh.subscribe("/can_tx", 10 , &kvaser_can_sender::callbackMobileyeCan, this);
		sub_mobileye_obstacle_data_ = nh.subscribe("/use_mobileye_obstacle", 10 , &kvaser_can_sender::callbackMobileyeObstacleData, this);
		sub_temporary_fixed_velocity_ = nh.subscribe("/temporary_fixed_velocity", 10 , &kvaser_can_sender::callbackTemporaryFixedVelocity, this);
		//sub_interface_config_ = nh_.subscribe("/config/microbus_interface", 10, &kvaser_can_sender::callbackConfigInterface, this);

		sub_current_pose_ = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, "/current_pose", 10);
		sub_current_velocity_ = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, "/current_velocity", 10);
		sync_twist_pose_ = new message_filters::Synchronizer<TwistPoseSync>(TwistPoseSync(SYNC_FRAMES), *sub_current_velocity_, *sub_current_pose_);
		sync_twist_pose_->registerCallback(boost::bind(&kvaser_can_sender::TwistPoseCallback, this, _1, _2));

		std::string safety_error_message = "";
		publishStatus(safety_error_message);

		waypoint_param_.blinker = 0;
		automatic_door_time_ = blinker_right_time_ = blinker_left_time_ =
		        blinker_stop_time_ = ros::Time::now();

		pid_params.init(0.0);
		mobileye_obstacle_data_.header.stamp = ros::Time(0);
	}

	~kvaser_can_sender()
	{
		delete sync_twist_pose_;
		delete sub_current_pose_;
		delete sub_current_velocity_;
	}

	const bool isOpen() {return kc.isOpen();}

	void can_send()
	{
		//if(can_receive_501_.emergency == false)
		{
			NdtGnssCheck();

			double current_velocity = 0;// = can_receive_502_.velocity_average / 100.0;

			switch(use_velocity_data_)
			{
			case USE_VELOCITY_CAN:
				current_velocity = can_receive_502_.velocity_average / 100.0;
				break;
			case USE_VELOCITY_TWIST:
				current_velocity = current_velocity_.twist.linear.x * 3.6;
				break;
			default:
				current_velocity = current_velocity_.twist.linear.x * 3.6;
				break;
			}

			double acceleration = 0;
			use_acceleration_data_ = USE_ACCELERATION_TWIST2;
			switch(use_acceleration_data_)
			{
			case USE_ACCELERATION_TWIST1:
				acceleration = acceleration1_twist_;
				break;
			case USE_ACCELERATION_TWIST2:
				acceleration = acceleration2_twist_;
				break;
			case USE_ACCELERATION_IMU:
				/*double ax = imu_.linear_acceleration.x;
				double ay = imu_.linear_acceleration.y;
				double az = imu_.linear_acceleration.z;
				acceleration = sqrt(ax*ax + ay*ay + az*az);*/
				acceleration = imu_.linear_acceleration.x;
				break;
			default:
				break;
			}

			unsigned char buf[SEND_DATA_SIZE] = {0,0,0,0,0,0,0,0};
			bufset_mode(buf);
			bufset_steer(buf);
			bufset_drive(buf, current_velocity, acceleration, 2.0);
			bufset_car_control(buf, current_velocity);

			autoware_msgs::VehicleStatus status;
			status.header.stamp = ros::Time::now();
			status.drivemode = (buf[0] & 0x0B != 0x0B) ? autoware_msgs::VehicleStatus::MODE_AUTO : autoware_msgs::VehicleStatus::MODE_MANUAL;
			status.steeringmode = (buf[0] & 0xA0 != 0xA0) ? autoware_msgs::VehicleStatus::MODE_AUTO : autoware_msgs::VehicleStatus::MODE_MANUAL;
			status.gearshift = 0;
			status.lamp = 0;
			status.light = 0;
			status.speed = current_velocity_.twist.linear.x;
			if(can_receive_502_.angle_actual > 0) status.angle = can_receive_502_.angle_actual / wheelrad_to_steering_can_value_left;
			else status.angle = can_receive_502_.angle_actual / wheelrad_to_steering_can_value_right;
			pub_vehicle_status_.publish(status);
			
			kc.write(0x100, (char*)buf, SEND_DATA_SIZE);
			loop_counter_++;
		}
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "kvaser_microbus_can_sender_stroke");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	kvaser_can_sender kcs(nh, private_nh);
	if(kcs.isOpen() == false)
	{
		std::cerr << "error : open" << std::endl;
	}

	//kcs.emergency_reset();
	ros::Rate loop_rate(100);
	while(ros::ok())
	{
		kcs.can_send();
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
