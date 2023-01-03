
#include "gaja_driver/RosGajaDriver.hpp"
#include "./SerialConnectionManager.c"

RosGajaDriver::RosGajaDriver(ros::NodeHandle& nodeHandle) :
		nodeHandle_(nodeHandle) {

	if(ConnectConfig("/dev/ttyUSB0"))
		ROS_INFO("Connect parameters configured...");
	else { ROS_ERROR("Connect configuration error"); ros::requestShutdown(); }

	ROS_INFO("Listening for commands...");
	engines = 0;
	firstCamSet = true;
	steeringCmdSub_ = nodeHandle_.subscribe("/gaja_steering", 5, &RosGajaDriver::CmdCallback, this);
	cameraSetSub_ = nodeHandle_.subscribe("/cam_config", 1, &RosGajaDriver::CameraCallback, this); 
	
	//ros::spin();
	//ROS_INFO("Constructor ready 2");
	// implement linuxCserialManager2.c
}

RosGajaDriver::~RosGajaDriver() {
}

bool RosGajaDriver::ConnectConfig(char portname[])
{

	// Open File Descriptor 
	USB = open( portname, O_RDWR | O_NOCTTY | O_SYNC ); // O_WRONLY

	// Error Handling 
	if ( USB < 0 )
	{
		printf("Error %d opening %s: %s", errno, portname, strerror(errno));
		return false;
	}
	
	set_interface_attribs (USB, B9600, 0);  // set speed to 9,600 bps, 8n1 (no parity)
	set_blocking (USB, 0);              	// set no blocking
	
	return true;
}

void RosGajaDriver::CmdCallback(const gaja_msg::Move_command::ConstPtr& msg)
{
	char data[60];
	std::string msg_dir = msg->direction.c_str();
	int msg_pwm = std::stoi( msg->pwm );

								// MOTORS:			LEFT F	RIGHT F		LEFT R	RIGHT R
	if(msg_dir == "UP")
		sprintf(data, " 128 0 %d 192 \n 128 1 %d 192 \n 128 2 %d 192 \n 128 3 %d 192", msg_pwm, msg_pwm, msg_pwm, msg_pwm);
	else if(msg_dir == "DOWN")
		sprintf(data, " 128 0 %d 192 \n 128 1 %d 192 \n 128 2 %d 192 \n 128 3 %d 192", -msg_pwm, -msg_pwm, -msg_pwm, -msg_pwm);
	else if(msg_dir == "LEFT")
		sprintf(data, " 128 0 %d 192 \n 128 1 %d 192 \n 128 2 %d 192 \n 128 3 %d 192", -msg_pwm, msg_pwm, -msg_pwm, msg_pwm);
	else if(msg_dir == "RIGHT")
		sprintf(data, " 128 0 %d 192 \n 128 1 %d 192 \n 128 2 %d 192 \n 128 3 %d 192", msg_pwm, -msg_pwm, msg_pwm, -msg_pwm);
	else if(msg_dir == "STOP")
		sprintf(data, " 128 0 20 192 \n 128 1 20 192 \n 128 2 20 192 \n 128 3 20 192");

	//ROS_INFO("I heard: 128 %d %s 192 - direction: %s", engines, msg->pwm.c_str(), msg->direction.c_str());

	int w = write (USB, data, strlen(data));    // send 'strlen(data)' character greeting
	if (w < 0)
  		ROS_ERROR("write() of data failed!\n");

	usleep ((strlen(data) + 25) * 100);   // sleep enough to transmit the 'strlen(result)' plus
                                     		// receive 25:  approx 100 uS per char transmit
	char buf [100];
	int n = read (USB, buf, sizeof buf);  	// read up to 100 characters if ready to read

	ROS_INFO("Buf value: %.4s\n", buf);
}


void RosGajaDriver::CameraCallback(const gaja_msg::Cam_config::ConstPtr& msg)
{
	ROS_INFO("*****************START OF CALLBACK************************");
	if (!firstCamSet) { system("ps -ef | sed -n '/camera_stream/{/grep/!p;}' | awk '{print$2}' | xargs -i kill {}"); sleep(4); }

	ROS_INFO("*****************START OF LAUNCH************************");
	char cam_data[100];
	sprintf(cam_data, "roslaunch web_interface camera_stream.launch cam_width:=%s cam_height:=%s pixel_format:=%s &", 
		msg->width.c_str(), msg->height.c_str(), msg->pixel_fmt.c_str());
	
	firstCamSet = false;
	
	system( cam_data );

	
	ROS_INFO("*******************END OF CALLBACK*************************");
}


