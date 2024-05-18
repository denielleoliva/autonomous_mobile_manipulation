#include <iostream>
#include <string>

#include <Eigen/QR>
#include <Eigen/Geometry>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TransformStamped.h"

class EEForcePub
{
private: 
	std::shared_ptr<KDL::ChainDynParam> dyn_solver;
	std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver;
	KDL::Tree robot_tree;
	KDL::Chain arm_chain;

	ros::Subscriber joint_sub;
	ros::Publisher force_pub;

	int target_link_index_in_chain;

	tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

public:

	EEForcePub(ros::NodeHandle& nh, std::string description_param, std::string chain_base_link, std::string chain_end_link, int ft_sensor_link_index) :
		tfListener(tfBuffer)
	{
		if (!kdl_parser::treeFromParam(description_param.c_str(), this->robot_tree))
		{
			ROS_ERROR("Oh no! tree wasn\'t parsed!");
			return;
		}

		if (!this->robot_tree.getChain(chain_base_link, chain_end_link, this->arm_chain))
		{
			ROS_ERROR("Oh no! couldn\'t parse the chain from %s to %s", chain_base_link.c_str(), chain_end_link.c_str());
			return;
		}

		KDL::Vector gravity(0.0, 0.0, -9.81);

		this->dyn_solver = std::make_shared<KDL::ChainDynParam>(this->arm_chain, gravity);
		this->jac_solver = std::make_shared<KDL::ChainJntToJacSolver>(this->arm_chain);

		ROS_INFO("The chain has %d joints.", this->arm_chain.getNrOfJoints());

		this->target_link_index_in_chain = ft_sensor_link_index;

		this->joint_sub = nh.subscribe("/bvr_SIM/joint_states", 1000, &EEForcePub::joint_callback, this);
		this->force_pub = nh.advertise<geometry_msgs::Vector3>("bvr_SIM/force_due_to_gravity", 1000);
	}

	void joint_callback(const sensor_msgs::JointState::ConstPtr& data)
	{
		(void) data;
		KDL::JntArray joint_state(this->arm_chain.getNrOfJoints());
		joint_state.data[0] = data->position[5];// shoulder_pan
		joint_state.data[1] = data->position[4];// shoulder_lift
		joint_state.data[2] = data->position[2];// elbow_joint
		joint_state.data[3] = data->position[6];// wrist_1_joint
		joint_state.data[4] = data->position[7];// wrist_2_joint
		joint_state.data[5] = data->position[8];// wrist_3_joint

		KDL::JntArray gravity_torques(this->arm_chain.getNrOfJoints());
		KDL::Jacobian jac(this->arm_chain.getNrOfJoints());

		this->dyn_solver->JntToGravity(joint_state, gravity_torques);
		this->jac_solver->JntToJac(joint_state, jac, this->target_link_index_in_chain);

		Eigen::MatrixXd jac_transpose_pinv = jac.data.transpose().completeOrthogonalDecomposition().pseudoInverse();

		Eigen::MatrixXd wrench = jac_transpose_pinv * gravity_torques.data;

		Eigen::MatrixXd test = jac.data.transpose() * wrench;

		// transform for the opposite of the torque we need (the force applied)
		wrench = wrench * -1;

		std::cout << "---------------------------------------------" << std::endl;
		std::cout << test - gravity_torques.data << std::endl << std::endl;

		// finally transform to the wrist_1_link frame
		geometry_msgs::TransformStamped tf_stamped;

		try
		{
			tf_stamped = this->tfBuffer.lookupTransform("bvr_SIM/main_arm_SIM/wrist_1_link", "bvr_SIM/main_arm_SIM/base_link", ros::Time(0));
		}
		catch (tf2::TransformException& ex)
		{
			ROS_ERROR("%s", ex.what());
			return;
		}

		Eigen::VectorXd tf_translation(3);

		tf_translation[0] = tf_stamped.transform.translation.x;
		tf_translation[1] = tf_stamped.transform.translation.y;
		tf_translation[2] = tf_stamped.transform.translation.z;
		// tf_translation[3] = 0.0;
		// tf_translation[4] = 0.0;
		// tf_translation[5] = 0.0;

		wrench = wrench + tf_translation;
		std::cout << wrench << std::endl << std::endl;

		Eigen::Quaterniond tf_rotation;

		tf_rotation.w() = tf_stamped.transform.rotation.w;
		tf_rotation.x() = tf_stamped.transform.rotation.x;
		tf_rotation.y() = tf_stamped.transform.rotation.y;
		tf_rotation.z() = tf_stamped.transform.rotation.z;

		Eigen::MatrixXd transformed_wrench = tf_rotation * wrench * .6;

		std::cout << transformed_wrench << std::endl << std::endl;

		geometry_msgs::Vector3 to_pub;

		to_pub.x = transformed_wrench(0, 0);
		to_pub.y = transformed_wrench(1, 0);
		to_pub.z = transformed_wrench(2, 0);

		this->force_pub.publish(to_pub);
	}
};

int main (int argc, char **argv)
{
	ros::init(argc, argv, "force_node");

	ros::NodeHandle nh;

	std::shared_ptr<EEForcePub> my_cp = std::make_shared<EEForcePub>(
												nh, 
												"bvr_SIM/robot_description",
												"bvr_SIM/main_arm_SIM/base_link", 
												"bvr_SIM/main_arm_SIM/gripper_manipulation_link",
												4
											);

	ros::spin();

	return 0;
}