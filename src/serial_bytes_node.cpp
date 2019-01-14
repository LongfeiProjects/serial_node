#include "ros/ros.h"
#include <serial/serial.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

serial::Serial ros_serial;

void serialCallback(const std_msgs::UInt8MultiArray msg)
{
	int tx_data_size = msg.layout.dim[0].size;
	uint8_t temp_data[tx_data_size];
	copy(msg.data.begin(), msg.data.end(), temp_data);
	ros_serial.write(temp_data, tx_data_size);
	ROS_INFO_STREAM("Write " << tx_data_size << " bytes to serial port");
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "serial_bytes_node");
	ros::NodeHandle nh;

	ros::Publisher serial_rx_pub = nh.advertise<std_msgs::UInt8MultiArray>("serial_rx", 256);
	ros::Subscriber serial_tx_sub = nh.subscribe("serial_tx", 256, serialCallback);

	try
	{
		ros_serial.setPort("/dev/ttyTHS2");
		ros_serial.setBaudrate(115200);
		serial::Timeout serial_timeout = serial::Timeout::simpleTimeout(1000);
		ros_serial.setTimeout(serial_timeout);
		ros_serial.open();
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}

	if (ros_serial.isOpen())
	{
		ROS_INFO_STREAM("Serial port opened");
	}
	else
	{
		return -1;
	}

	ros::Rate loopRate(200);
	while (ros::ok())
	{
		ros::spinOnce();

		if (ros_serial.available())
		{
			std_msgs::UInt8MultiArray serial_rx_data;
			int rx_data_size = ros_serial.available();
			ros_serial.read(serial_rx_data.data, rx_data_size);
			ROS_INFO_STREAM("Read " << rx_data_size << " bytes from serial port");

			serial_rx_data.layout.dim.push_back(std_msgs::MultiArrayDimension());
			serial_rx_data.layout.dim[0].size = rx_data_size;
			serial_rx_pub.publish(serial_rx_data);
		}

		loopRate.sleep();
	}

	return 0;
}
