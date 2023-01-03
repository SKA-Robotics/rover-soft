#pragma once

#include "gaja_msg/Move_command.h"
#include "gaja_msg/Cam_config.h"
#include <ros/ros.h>
#include <string.h>


class RosGajaDriver {
	public:
		/** Constructor */
		RosGajaDriver(ros::NodeHandle& nodeHandle);
		
		bool ConnectConfig(char portname[]);

		void CmdCallback(const gaja_msg::Move_command::ConstPtr& msg);
		
		void CameraCallback(const gaja_msg::Cam_config::ConstPtr& msg);
		/** Destructor */
		virtual ~RosGajaDriver();

	private:
		//bool readParameters();
		int USB, engines;
		ros::NodeHandle nodeHandle_;
		ros::Subscriber steeringCmdSub_;
		ros::Subscriber cameraSetSub_;
		bool firstCamSet;
		//ros::Rate loopRate_;
		//std::string scanTopic_;
		//int subscriberQueueSize_;

};
