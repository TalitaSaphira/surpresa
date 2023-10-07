#include <iostream>
#include <cfloat>
#include <cstring>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>

using namespace ros;
using namespace std;

std::vector<float> vec;

void RouteCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	for (const auto& num : msg->data){
		vec.push_back(num);
        //std::cout << "Caminho recebido \n";
        ROS_INFO("Caminho recebido\n");
	}
}

int main (int argc, char **argv) {

    ros::init(argc, argv, "movimentation_main_node");
	ros::NodeHandle nh;

    ros::Publisher robot_position_pub = nh.advertise<std_msgs::Float32MultiArray>("robot_position", 1000);
    ros::Publisher robot_destination_pub = nh.advertise<std_msgs::Float32MultiArray>("robot_destination", 1000);

    Rate loop_rate(0.5);

    int count = 0;

    while(ok()){
        //====Dados de teste======================
        std_msgs::Float32MultiArray rob_pos;
        std_msgs::Float32MultiArray rob_des;

        rob_pos.data = {6, 6};
        rob_des.data = {0, 1};
        //========================================

        robot_position_pub.publish(rob_pos);
        robot_destination_pub.publish(rob_des);

        ros::Subscriber rota_sub = nh.subscribe("pnts_vetor", 1000, RouteCallback);

        ros::spinOnce();
        loop_rate.sleep();

        count++;
	}

    return 0;
}