/*
 * vmu931-read-all example for vmu931 library
 *
 * Copyright 2018 (C) Bartosz Meglicki <meglickib@gmail.com>
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. 
 *
 */
 
 /*
  * This example:
  * - initilies communication with VMU931
  * - enables euler, magnetometer and accelerometer data streams
  * - reads & prints data from VMU931 1000 times (+- 5 seconds)
  * - cleans after itself
  * 
  * Program expects terminal device, e.g.
  * 
  * ./vmu931-read-all /dev/ttyACM0
  * 
  *
  */

#include "vmu931.h"

#include <stdio.h> //printf
#include <stdlib.h> //exit

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include <sstream>

void usage(char **argv);

const int DATA_SIZE=10; 
const int MAX_READS=1000;


int main(int argc, char **argv)
{
	// imu_msg = Imu();
	sensor_msgs::Imu imu_msg;

	ros::init(argc, argv, "imu_node");

	ros::NodeHandle n;

	ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("imu", 1000);
	

	struct vmu *vmu=NULL;
	// suppose we are intersted in euler, magnetometer and accelerometer
	// note that quaternions and heading use different data types
	struct vmu_txyz euler_data[DATA_SIZE];
	struct vmu_txyz mag_data[DATA_SIZE];
	struct vmu_txyz accel_data[DATA_SIZE];
	struct vmu_twxyz quat_data[DATA_SIZE];
	struct vmu_txyz gyro_data[DATA_SIZE];

	struct vmu_data data={0}; //the library will return data here and note number of read values in data.size
	struct vmu_size size={0}; //this notes sizes of our arrays, data.size is refreshed with this value before read

	size.accel = size.mag = size.euler = size.quat = size.gyro = DATA_SIZE;	

	data.euler = euler_data;
	data.mag = mag_data;
	data.accel = accel_data;
	data.quat = quat_data;
	data.gyro = gyro_data;
	data.size = size;

	const char *tty_device=argv[1];
	int ret, reads=0;
	
	if(argc != 2)
	{
		usage(argv);
		return EXIT_SUCCESS;
	}
	
	if( (vmu=vmu_init(tty_device)) == NULL)
	{
		perror(	"unable to initialize VMU931\n\n"
				"hints:\n"
				"- it takes a few seconds after plugging in to initialize device\n"
				"- make sure you are using correct tty device (dmesg after plugging vmu)\n"
				"- if all else fails unplug/plug VMU931\n\n"
				"error details");
		return 1;
	}	
	
	if( vmu_stream(vmu, VMU_STREAM_EULER | VMU_STREAM_MAG | VMU_STREAM_ACCEL | VMU_STREAM_QUAT | VMU_STREAM_GYRO) == VMU_ERROR )
	{
		perror("failed to stream euler/mag/accel data");
		exit(EXIT_FAILURE);
	}
		
	while( (ret=vmu_read_all(vmu, &data)) != VMU_ERROR )
	{
		imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";

		for(int i=0;i<data.size.euler;++i)
		{
			printf("[euler] t=%u x=%f y=%f z=%f\n", data.euler[i].timestamp_ms, data.euler[i].x, data.euler[i].y, data.euler[i].z);
			
		}

		for(int i=0;i<data.size.mag;++i)
			printf("[mag] t=%u x=%f y=%f z=%f\n", data.mag[i].timestamp_ms, data.mag[i].x, data.mag[i].y, data.mag[i].z);

		for(int i=0;i<data.size.accel;++i)
		{
			printf("[accel] t=%u x=%f y=%f z=%f\n", data.accel[i].timestamp_ms, data.accel[i].x, data.accel[i].y, data.accel[i].z);
			imu_msg.linear_acceleration.x = data.accel[i].x * 3.14/180;
        	imu_msg.linear_acceleration.y = data.accel[i].y * 3.14/180;
        	imu_msg.linear_acceleration.z = data.accel[i].z * 3.14/180;
		}

		for(int i=0;i<data.size.quat;++i)
		{
			printf("[quat] t=%u w=%f x=%f y=%f z=%f\n", data.quat[i].timestamp_ms, data.quat[i].w, data.quat[i].x, data.quat[i].y, data.quat[i].z);
			imu_msg.orientation.x = data.quat[i].x;
        	imu_msg.orientation.y = data.quat[i].y;
        	imu_msg.orientation.z = data.quat[i].z;
        	imu_msg.orientation.w = data.quat[i].w;

			// imu_msg.linear_acceleration.x = data.quat[i].x; //bug
        	// imu_msg.linear_acceleration.y = data.quat[i].y; //bug
        	// imu_msg.linear_acceleration.z = data.quat[i].z; //bug

			// imu_msg.angular_velocity.x = data.quat[i].x;  //bug
        	// imu_msg.angular_velocity.y = data.quat[i].y;  //bug
        	// imu_msg.angular_velocity.z = data.quat[i].z;  //bug
		}

		for(int i=0;i<data.size.gyro;++i)
		{
			printf("[gyro] t=%u x=%f y=%f z=%f\n", data.gyro[i].timestamp_ms, data.gyro[i].x, data.gyro[i].y, data.gyro[i].z);
			imu_msg.angular_velocity.x = data.gyro[i].x  * 3.14/180;
        	imu_msg.angular_velocity.y = data.gyro[i].y  * 3.14/180;
        	imu_msg.angular_velocity.z = data.gyro[i].z  * 3.14/180;
		}

		//terminate after reading MAX_READS times
		//remove those lines if you want to read infinitely
		// if(++reads >= MAX_READS) 
		// 	break;
			
		//refresh the sizes of the arrays for data streams
		data.size=size;

		imu_pub.publish(imu_msg);
	}
		
	if(ret == VMU_ERROR)
		perror("failed to read from VMU931");
	else
		printf("success reading from VMU931, bye...\n");
	
	vmu_close(vmu);
	
	return 0;
}

void usage(char **argv)
{
	printf("Usage:\n");
	printf("%s tty_device\n\n", argv[0]);
	printf("examples:\n");
	printf("%s /dev/ttyACM0\n", argv[0]);
}
