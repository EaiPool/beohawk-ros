#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <art_arduino/PIDConfig.h>
#include <art_arduino/PIDConstants.h>

using namespace std;

class PIDTuning
{
	ros::NodeHandle& nh;
	dynamic_reconfigure::Server<art_arduino::PIDConfig> srv;
	art_arduino::PIDConstants pid_constants;

	ros::Publisher pub_constants;

	void reconfigure_callback(art_arduino::PIDConfig &config, uint32_t level)
	{
		if(!ros::ok()) return;

		bool updated = false;
		
		pid_constants.proll = static_cast<float>(config.proll);
		pid_constants.iroll = static_cast<float>(config.iroll);
		pid_constants.droll = static_cast<float>(config.droll);
		pid_constants.ppitch = static_cast<float>(config.ppitch);
		pid_constants.ipitch = static_cast<float>(config.ipitch);
		pid_constants.dpitch = static_cast<float>(config.dpitch);
		pid_constants.pyaw = static_cast<float>(config.pyaw);
		pid_constants.iyaw = static_cast<float>(config.iyaw);
		pid_constants.dyaw = static_cast<float>(config.dyaw);
		pid_constants.paltitude = static_cast<float>(config.paltitude);
		pid_constants.ialtitude = static_cast<float>(config.ialtitude);
		pid_constants.daltitude = static_cast<float>(config.daltitude);
		pid_constants.pxpose = static_cast<float>(config.pxpose);
		pid_constants.ixpose = static_cast<float>(config.ixpose);
		pid_constants.dxpose = static_cast<float>(config.dxpose);
		pid_constants.pypose = static_cast<float>(config.pypose);
		pid_constants.iypose = static_cast<float>(config.iypose);
		pid_constants.dypose = static_cast<float>(config.dypose);
		pid_constants.pthetapose = static_cast<float>(config.pthetapose);
		pid_constants.ithetapose = static_cast<float>(config.ithetapose);
		pid_constants.dthetapose = static_cast<float>(config.dthetapose);

		pub_constants.publish(pid_constants);
		
		system("rosparam dump $(rospack find art_arduino)/pid.yaml /arduino_pid_tuning_node");
	}
	
  public:
	PIDTuning(ros::NodeHandle& _nh): nh(_nh)
	{
		pub_constants = nh.advertise<art_arduino::PIDConstants>("/arduino/pid_constants", 10);
		dynamic_reconfigure::Server<art_arduino::PIDConfig>::CallbackType f;
		f = boost::bind(&PIDTuning::reconfigure_callback, this, _1, _2);
		srv.setCallback(f);
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arduino_pid_tuning_node");
	ros::NodeHandle nh;
	PIDTuning pid_tuning(nh);
	ros::spin();
	return 0;
}
