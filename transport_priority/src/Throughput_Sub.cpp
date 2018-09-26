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

class ThroughtputSub : public rclcpp::Node
{
public:
    ThroughtputSub(std::string topic) : Node("throughput_sub")
	{
		subscription_ = this->create_subscription<std_msgs::msg::String>(topic, 
                        std::bind(&ThroughtputSub::topic_callback, this, _1));
		cnt_ = 0;
	}


private:
	int cnt_;
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        (void)msg;
		
		cnt_++;
		
		if (cnt_ % 1000 == 0)
		{
			std::cout << "Recieved the " << cnt_ << "th msg." << std::endl;
		}	
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

void print_usage()
{
    printf("ROS2 transport priority testing tool - Sub:\n");
    printf("options:\n");
    printf("-no : topic no. [default=1].\n");
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    printf("ROS2 transport priority testing tool - Sub:\n");
		
	
	// topic name
	std::string no = "1";
	if (rcutils_cli_option_exist(argv, argv + argc, "-no"))
	{
		no = rcutils_cli_get_option(argv, argv + argc, "-no");
	}
	
	std::string topic = "tp_" + no;
    printf("Subscribing to topic \"%s\"\n", topic.c_str());
	
	// node
	rclcpp::spin(std::make_shared<ThroughtputSub>(topic));
    rclcpp::shutdown();	
    return 0;
}
