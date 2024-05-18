#include "effector_control/force_application.hh"

namespace gazebo
{

	ForceApplication::ForceApplication() : WorldPlugin(), spinner(1)
	{
		int argc;
		char **argv;

		
		ros::NodeHandle n;

		this->force_sub = n.subscribe("/gazebo_force_app", 1000, &ForceApplication::force_callback, this);

		this->node = transport::NodePtr(new transport::Node());
		this->node->Init();

		this->userCmdPub =  this->node->Advertise<msgs::UserCmd>("~/user_cmd");

		this->spinner.start();

		ROS_FATAL("hiiieeeee");
	}

	ForceApplication::~ForceApplication()
	{
		this->spinner.stop();
	}

	void ForceApplication::force_callback(const geometry_msgs::WrenchStamped::ConstPtr& data)
	{
		msgs::Wrench msg = this->ros_wrench_msg_to_msg(data);
		std::string linkName = "bvr_SIM/main_arm_SIM/wrist_3_link"; //data->header.frame_id;

		ROS_FATAL("here :)");

		// Register user command on server
		// The wrench will be applied from the server
		msgs::UserCmd userCmdMsg;
		userCmdMsg.set_description("Apply wrench to [" + linkName + "]");
		userCmdMsg.set_entity_name(linkName);
		userCmdMsg.set_type(msgs::UserCmd::WRENCH);
		userCmdMsg.mutable_wrench()->CopyFrom(msg);

		this->userCmdPub->Publish(userCmdMsg);
	}

	msgs::Wrench ForceApplication::ros_wrench_msg_to_msg(const geometry_msgs::WrenchStamped::ConstPtr& data)
	{
		msgs::Wrench to_ret;

		float x = 1000;//data->wrench.force.x;
		float y = 0;//data->wrench.force.y;
		float z = 0;//data->wrench.force.z;

		ROS_FATAL_STREAM(x << " " << y);

		float phi = data->wrench.torque.x;
		float omega = data->wrench.torque.y;
		float psi = data->wrench.torque.z;

		ignition::math::Vector3d forceVector(x, y, z);
		ignition::math::Vector3d torqueVector(phi, omega, psi);

		msgs::Set(to_ret.mutable_force(), forceVector);
		msgs::Set(to_ret.mutable_torque(), torqueVector);
		msgs::Set(to_ret.mutable_force_offset(), ignition::math::Vector3d::Zero);

		return to_ret;
	}
}