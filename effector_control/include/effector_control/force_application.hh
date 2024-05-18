#ifndef FORCE_APPLICATION_HH_
#define FORCE_APPLICATION_HH_

#include "ros/ros.h"

#include <gazebo/gazebo.hh>
// #include <gazebo/gui/ApplyWrenchDialogPrivate.hh>
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include <geometry_msgs/WrenchStamped.h>
#include <ignition/math/Helpers.hh>

namespace gazebo
{

	class ForceApplication : public WorldPlugin
	{
	public:
		ForceApplication();
		~ForceApplication();

		void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){};

	private:

		void force_callback(const geometry_msgs::WrenchStamped::ConstPtr& data);

		msgs::Wrench ros_wrench_msg_to_msg(const geometry_msgs::WrenchStamped::ConstPtr& data);

		ros::Subscriber force_sub;
        // std::unique_ptr<ApplyWrenchDialogPrivate> dataPtr;

        transport::NodePtr node;
        transport::PublisherPtr userCmdPub;

        ros::AsyncSpinner spinner;
	};

	GZ_REGISTER_WORLD_PLUGIN(ForceApplication)
}

#endif //FORCE_APPLICATION_HH_