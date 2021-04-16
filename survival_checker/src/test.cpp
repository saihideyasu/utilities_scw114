#include <ros/ros.h>
#include <std_msgs/String.h>
#include <topic_tools/shape_shifter.h>
#include <ros/master.h>

/*void callback(const std_msgs::String::ConstPtr &msg)
{
    std::cout << msg->data << std::endl;
}*/
void callback(const topic_tools::ShapeShifter::ConstPtr& msg)
{
    std::cout << "type : " << msg->getDataType() << std::endl;
    //std::cout << "definition : " << msg->getMessageDefinition() << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

    ros::Subscriber sub = nh.subscribe("/aaa", 1, callback);
    ros::spin();
    return 0;
}