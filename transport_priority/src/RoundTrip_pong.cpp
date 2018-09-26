// Copyright 2017 ADLINK Technology, Inc.
// Developer: HaoChih, LIN (haochih.lin@adlinktech.com)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h" 
#include "adlink_msgs/msg/ping_pong.hpp"

using std::placeholders::_1;

class RoundTripPONG : public rclcpp::Node
{
    public:
    RoundTripPONG(int argc, char * argv[]) : Node("roundtrip_pong"), count_(0)
    {
        
        // priority
        priority_ = 1;
        if (rcutils_cli_option_exist(argv, argv + argc, "-pr"))
        {
            priority_ = atoi(rcutils_cli_get_option(argv, argv + argc, "-pr"));
			priority_ = (priority_ > 2)? 2 : priority_;
			priority_ = (priority_ < 0)? 0 : priority_;
        }
		
        rmw_qos_profile_t profile = rmw_qos_profile_default;
		if (priority_ == 2) profile = rmw_qos_profile_high_priority;
		else if (priority_ == 0) profile = rmw_qos_profile_low_priority;
		
		
		std::string pub_topic = "rt_" + std::to_string(priority_) + "_pong";
		std::string sub_topic = "rt_" + std::to_string(priority_) + "_ping";
		printf("Publishing to topic \"%s\"\n", pub_topic.c_str());		
		printf("Subscribing to topic \"%s\"\n", sub_topic.c_str());
		
        publisher_ = this->create_publisher<adlink_msgs::msg::PingPong>(pub_topic, profile); //topic, QoS
        subscription_ = this->create_subscription<adlink_msgs::msg::PingPong>(sub_topic, 
                        std::bind(&RoundTripPONG::topic_callback, this, _1));
    }

private:
    void topic_callback(const adlink_msgs::msg::PingPong::SharedPtr msg)
    {
        msg_back_ = *msg;
        msg_back_.sender_id = 1;
        msg_back_.recver_id = msg->sender_id;         
        publisher_->publish(msg_back_);
        //std::cout << "I heard: " << msg->data.c_str() << ", Id: " << count_ << std::endl; //debug
        count_ ++;
    }

    rclcpp::Publisher<adlink_msgs::msg::PingPong>::SharedPtr publisher_;
    rclcpp::Subscription<adlink_msgs::msg::PingPong>::SharedPtr subscription_;
    adlink_msgs::msg::PingPong msg_back_;
    size_t count_;
	int priority_;
};


void print_usage()
{
    printf("ROS2 RoundTrip testing tool:\n");
    printf("options:\n");
    printf("-h : Print this help function.\n");
    printf("-pr : priority. Specify the priority of the publisher. [default=1].\n");
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Print the help function
    if (rcutils_cli_option_exist(argv, argv + argc, "-h")) 
    {
        print_usage();
        return 0;
    }

    std::cout << "RoundTrip Testing: pong side !" << std::endl;
    auto pong = std::make_shared<RoundTripPONG>(argc, argv);
    rclcpp::spin(pong);
    rclcpp::shutdown();
    return 0;
}
