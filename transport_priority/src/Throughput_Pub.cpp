// Copyright 2018 ADLINK Technology, Inc.
// Developer: Alan, CHEN (alan.chen@adlinktech.com)
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

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;

class ThroughtputPub : public rclcpp::Node
{
public:
    ThroughtputPub(std::string topic) : Node("throughput_pub"), cnt_(0)
	{
		
        publisher_ = this->create_publisher<std_msgs::msg::String>(topic);
		
		timer_ = this->create_wall_timer(
			milliseconds(1), std::bind(&ThroughtputPub::timer_callback, this));
    }


private:
    // Callback function for timer
    void timer_callback()
    {
		auto pub_msg = std_msgs::msg::String();
		pub_msg.data = std::string(4096, ' ');
		publisher_->publish(pub_msg);
		
		if (++cnt_ % 1000 == 0)
		{
			std::cout << "sent the " << cnt_ << "th msg..." << std::endl;
		}
    } // end of func
    
    int cnt_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

void print_usage()
{
    printf("ROS2 transport priority testing tool - PUB:\n");
    printf("options:\n");
    printf("-no : topic no. [default=1].\n");
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    printf("ROS2 transport priority testing tool - PUB:\n");
    printf("Message Size 4 KB & Publish Rate 1000 Hz:\n");
	
	if (rcutils_cli_option_exist(argv, argv + argc, "-h"))
	{
		print_usage();
		return 0;
	}
	
	// topic name	
	std::string no = "1";
	if (rcutils_cli_option_exist(argv, argv + argc, "-no"))
	{
		no = rcutils_cli_get_option(argv, argv + argc, "-no");
	}
	
	std::string topic = "tp_" + no;
    printf("Publishing to topic \"%s\"\n", topic.c_str());
	
	rclcpp::spin(std::make_shared<ThroughtputPub>(topic));
    rclcpp::shutdown();	
    return 0;
}
