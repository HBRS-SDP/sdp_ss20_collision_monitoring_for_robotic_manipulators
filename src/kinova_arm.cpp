#include "kinova_arm.h"

using namespace KDL;

double d2r(double v) {
	return v / 180 * M_PI;
}

KinovaArm::KinovaArm(std::string urdf_filename){
    // Import the tree from urdf
	// pulled from https://wiki.ros.org/kdl_parser/Tutorials/Start%20using%20the%20KDL%20parser
	if (!kdl_parser::treeFromFile(urdf_filename, arm_tree)){
		std::cout << "Failed to construct kdl tree" << std::endl;
	}
	arm_tree.getChain("base_link", "EndEffector_Link", chain);


	nr_joints = chain.getNrOfJoints();

	for(int i = 0; i < nr_joints; i++)
	{
		KDL::Frame* pose = new KDL::Frame();
		poses.push_back(pose);
	}

	#ifdef DEBUG
		std::cout << "num_joints: " << nr_joints << std::endl;
	#endif //DEBUG
}

KinovaArm::~KinovaArm(){

	// delete the pose frames
	for(int i = 0; i < nr_joints; i++)
	{
		delete(poses[i]);
	}

}

bool KinovaArm::updatePose(std::vector<double> joint_positions){

    // initailise the chain solver and the joint array
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
	JntArray jointpositions = JntArray(nr_joints);

	// pass the joint angles from function input into the joint array
	for(int i=0; i<nr_joints; i++)
	{
		jointpositions(i) = d2r(joint_positions[i]);
	}


	// solve for the frame at the "link" of the chain for the given joint positions
	for(int link_num = 0; link_num < nr_joints; link_num++)
	{
		if(fksolver.JntToCart(jointpositions, pos, link_num) >= 0)
		{
			#ifdef DEBUG
			std::cout << "Calculations to link number: " << link_num << std::endl 
			          << pos << std::endl
					  << "Success" << std::endl;
			#endif //DEBUG
		}
		// If calculation fails print error
		else
		{
			std::cout << "Error: could not calculate forward kinematics" << std::endl;
			return false;
		}
	}

	return true;
}