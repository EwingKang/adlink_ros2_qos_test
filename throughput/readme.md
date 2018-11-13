# ADLINK ROS 2.0 Performance Testing Tools  
  
## Developers
* Alan Chen (alan.chen@adlinktech.com)  

## License  
Apache 2.0 (Copyright 2017 ADLINK Technology, Inc.)  
  
# Dependencies  
* adlink_ros2_msgs  

## Guide
[Neuron A]

// Thoughput

ros2 run adlink_ros2_qos_test_priority Throughput_Pub -no 1

ros2 run adlink_ros2_qos_test_priority Throughput_Sub -no 2

// Ping

ros2 run adlink_ros2_qos_test_priority RoundTrip_ping -d -t 100000 -p 4 -pr 2 -f 1000

ros2 run adlink_ros2_qos_test_priority RoundTrip_ping -d -t 100000 -p 4 -pr 1 -f 1000

ros2 run adlink_ros2_qos_test_priority RoundTrip_ping -d -t 100000 -p 4 -pr 0 -f 1000


[Neuron B]

// Thoughput

ros2 run adlink_ros2_qos_test_priority Throughput_Pub -no 2

ros2 run adlink_ros2_qos_test_priority Throughput_Sub -no 1

// Pong

ros2 run adlink_ros2_qos_test_priority RoundTrip_pong -pr 2

ros2 run adlink_ros2_qos_test_priority RoundTrip_pong -pr 1

ros2 run adlink_ros2_qos_test_priority RoundTrip_pong -pr 0