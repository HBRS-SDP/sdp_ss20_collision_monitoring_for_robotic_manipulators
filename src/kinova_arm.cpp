#include "kinova_arm.h"


KinovaArm::KinovaArm(std::string urdf_filename){
    // Import the tree from urdf
	// pulled from https://wiki.ros.org/kdl_parser/Tutorials/Start%20using%20the%20KDL%20parser
	// if (!kdl_parser::treeFromFile(urdf_filename, arm_tree)){
	// 	std::cout << "Failed to construct kdl tree" << std::endl;
	// }
	// arm_tree.getChain("base_link", "EndEffector_Link", chain);


	// nr_joints = chain.getNrOfJoints();
}

KinovaArm::~KinovaArm(){

}

bool KinovaArm::updatePose(std::vector<double>){

    return true;
};