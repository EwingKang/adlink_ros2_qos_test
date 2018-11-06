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

class ThroughtputPub : public rclcpp::Node
{
public:
    ThroughtputPub(std::string topic, int payload_size, float timer_rate) : Node("throughput_pub"), cnt_(0)
	{
		payload_size_ = payload_size;
		timer_rate_ = timer_rate;
		//no_delay_ = no_delay;
		rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
		custom_qos.reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
        publisher_ = this->create_publisher<std_msgs::msg::String>(topic, custom_qos);
		
		last_clock_ = std::chrono::system_clock::now();
		
		if (timer_rate_ > 0 )
		{
			timer_ = this->create_wall_timer(
									microseconds(int((float)1000000/timer_rate_)),
									std::bind(&ThroughtputPub::timer_callback, this)  );
		}else if (timer_rate_ == -1)
		{
			timer_ = this->create_wall_timer(
									microseconds(0),
									std::bind(&ThroughtputPub::no_delay_callback, this)  );
		}
		/*
		if( !no_delay )
		{
			last_clock_ = std::chrono::system_clock::now();
			timer_ = this->create_wall_timer(
									microseconds(int((float)1000000/timer_rate_)),
									std::bind(&ThroughtputPub::timer_callback, this)  );
		}else {
			this->no_delay_loop();
		}*/
    }


private:
    // Callback function for timer
    void timer_callback()
    {
		auto pub_msg = std_msgs::msg::String();
		
		pub_msg.data = std::string(payload_size_, 'a');
		publisher_->publish(pub_msg);
		
		if (++cnt_ % (int)timer_rate_ == 0)
		{
			auto now = std::chrono::system_clock::now();
			std::chrono::duration<double> duration_sec = now - last_clock_;
			std::cout << std::fixed << std::setprecision(3)
					  << "sent the " << cnt_ << "th msg at " 
					  << ((double)timer_rate_/duration_sec.count())
					  << " Hz, " << (float)payload_size_*timer_rate_/1024/duration_sec.count() << "KB/s (" << (float)payload_size_*timer_rate_*8/duration_sec.count() << " bps)"<< std::endl;
			last_clock_ = now;
		}
    } // end of func
    
    
    void no_delay_callback()
	{
		auto pub_msg = std_msgs::msg::String();
		pub_msg.data = std::string(payload_size_, ' ');
		
		publisher_->publish(pub_msg);
		cnt_++;
		
		auto now = std::chrono::system_clock::now();
		std::chrono::duration<double> duration_sec = now - last_clock_;
		if ( duration_sec.count() >=1 ) // for every second
		{
			int cnt_delta = cnt_ - last_cnt_;
			std::cout << std::fixed << std::setprecision(3)
					<< "sent the " << cnt_ << "th msg at " 
					<< ((float)cnt_delta/duration_sec.count())  << " Hz, " 
					<< (float)payload_size_*cnt_delta/1024/duration_sec.count() << "KB/s (" 
					<< (float)payload_size_*cnt_delta*8/duration_sec.count() << " bps)"
					<< std::endl;
			last_clock_ = now;
			last_cnt_ = cnt_;
		}
	}
    
    int cnt_ = 0, last_cnt_ = 0;
	int payload_size_ = 4096;
	float timer_rate_ = 1000;
	//bool no_delay_ = false;		// for future spin_node_once update
	std::chrono::time_point<std::chrono::system_clock> last_clock_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

void print_usage()
{
    printf("ROS2 transport priority testing tool - PUB:\n");
    printf("options:\n");
    printf("-no : topic no. [default=1].\n");
	printf("-s : size in byte [default=4096].\n");
	printf("-r : target publishing rate, -1 for instant loop [default=1000].\n");
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    	
	// Parsing command options
	if (rcutils_cli_option_exist(argv, argv + argc, "-h"))
	{
		print_usage();
		return 0;
	}
	std::string no = "1"; // suffix of topic name
	float timer_rate = 1000;
	int payload_size = 4096;
	if (rcutils_cli_option_exist(argv, argv + argc, "-no"))
	{
		no = rcutils_cli_get_option(argv, argv + argc, "-no");
	}
	if (rcutils_cli_option_exist(argv, argv + argc, "-s"))
	{
		payload_size = atoi(rcutils_cli_get_option(argv, argv + argc, "-s") );
	}
	if (rcutils_cli_option_exist(argv, argv + argc, "-r"))
	{
		timer_rate = atof(rcutils_cli_get_option(argv, argv + argc, "-r") );
	}
	
	std::string topic = "tp_" + no;
	
	std::cout <<"ROS2 transport priority testing tool - PUBLISHER" << std::endl;
	std::cout << "Publishing to topic " << topic << std::endl;
	std::cout << "Message Size " << payload_size/1024 << "KB, ( " << payload_size*8 << " bit )"
			  << ", Publish at " << timer_rate << " Hz:" << std::endl;
	rclcpp::spin(std::make_shared<ThroughtputPub>(topic, payload_size, timer_rate));
		
	/* for future spin_node_once update
	if( timer_rate > 0 ) 
	{
		
		rclcpp::spin(node);	
		
	}else {
		no_delay = true;
		std::cout << "Message Size " << payload_size_/1024 << "KB, ( " << payload_size_*8 << " bit )"
				  << ", Publish without delay" << std::endl;
		
		auto node = std::make_shared<ThroughtputPub>(topic);
		while(1) 
		{
			spin_node_once()
		}
	}*/
	
	
    rclcpp::shutdown();	
    return 0;
}
