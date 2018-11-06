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
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;
using namespace std::chrono;
using std::placeholders::_1;

class ThroughtputSub : public rclcpp::Node
{
public:
    int TestCnt;
    ThroughtputSub(std::string topic) : Node("throughput_sub")
	{
		TestCnt = 0;
		last_clock_ = high_resolution_clock::now();
		cnt_ = 0;
		last_count_ = 0;
		isFirstClock_ = true;
		
		subscription_ = this->create_subscription<std_msgs::msg::String>(topic, 
                        std::bind(&ThroughtputSub::topic_callback, this, _1), 
						rmw_qos_profile_default
																		);
	}


private:
    int cnt_, last_count_;
	int payload_size_;
    bool isFirstClock_;
	std::chrono::time_point<std::chrono::system_clock> last_clock_;
	rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
	
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        //(void)msg; // to elimate unused message
		cnt_++;
		
		auto now = std::chrono::system_clock::now();
		std::chrono::duration<double> duration_sec = now - last_clock_;
		
		if (duration_sec.count() >=1 )
		{
			payload_size_ = msg->data.length();
			int cnt_delta = cnt_ - last_count_;
			std::cout << std::fixed << std::setprecision(3) 
					  << "count " << cnt_delta << " message, at " 
					  << (float)cnt_delta / duration_sec.count() << " Hz, data rate "
					  << (float)payload_size_*cnt_delta/1024/duration_sec.count() << "KB/s (" 
					  << (float)payload_size_*cnt_delta*8/duration_sec.count() << " bps)"
					  << std::endl;
			last_clock_ = now;
			last_count_ = cnt_;
			if( isFirstClock_ ) 
			{
				std::cout << "Message Size " << payload_size_/1024 << "KB, ( " 
						  << payload_size_*8 << " bit )" << std::endl;
				isFirstClock_ = false;
			}
			
		}
			
			
	}

};

void print_usage()
{
    printf("ROS2 transport priority testing tool - Sub:\n");
    printf("options:\n");
    printf("-no : topic no. [default=1].\n");
	printf("-s : size in byte [default=4096].\n");
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
    printf("Subscirbing to topic \"%s\"\n", topic.c_str());
	
	// node
	rclcpp::spin(std::make_shared<ThroughtputSub>(topic));
    rclcpp::shutdown();	
    return 0;
}
