#include "kinova_arm.h"

using namespace KDL;

KinovaArm::KinovaArm(std::string urdf_filename){
    // Import the tree from urdf
	// pulled from https://wiki.ros.org/kdl_parser/Tutorials/Start%20using%20the%20KDL%20parser
	if (!kdl_parser::treeFromFile(urdf_filename, arm_tree)){
		std::cout << "Failed to construct kdl tree" << std::endl;
	}
	arm_tree.getChain("base_link", "EndEffector_Link", chain);


	nr_joints = chain.getNrOfJoints();
}

KinovaArm::~KinovaArm(){

}

bool KinovaArm::updatePose(std::vector<double> joint_positions){

    // initailise the chain solver and the joint array
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
	JntArray jointpositions = JntArray(nr_joints);

	// pass the joint angles from function input into the joint array
	for(int i=0; i<nr_joints; i++)
	{
		jointpositions(i) = joint_positions[i];
	}


	// solve for the frame at the "link" of the chain for the given joint positions
	for(int link_num = 0; link_num < nr_joints; link_num++)
	{
		if(fksolver.JntToCart(jointpositions, pos, link_num) >= 0)
		{
			std::cout << "Calculations to link number: " << link << std::endl << pos << std::endl
					<< "Success" << std::endl;
			return true;
		}
		// If calculation fails print error
		else
		{
			std::cout << "Error: could not calculate forward kinematics" << std::endl;
			return false;
		}
	}
}