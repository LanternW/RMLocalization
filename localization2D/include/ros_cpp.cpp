#include <ros/ros.h> 
#include <geometry_msgs/Quaternion.h>


void callBack(geometry_msgs::Quaternion::ConstPtr& msg)
{
    
}

void timerCallBack(const ros::TimerEvent& e)
{
}

int main (int argc, char** argv) 
{ 
    ros::init(argc, argv, "node_name"); 
    ros::NodeHandle nh; 

    ros::Subscriber sub = nh.subscribe("/topic_name", 1000, callBack); 
    ros::Publisher pub = nh.advertise<geometry_msgs::Quaternion>("/topic_name_", 5); 
    ros::Timer timer = nh.createTimer(ros::Duration(0.1), timerCallBack);

    ros::spin();
    return 0;
}
