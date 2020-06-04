#include "kinova_arm.h"

using namespace KDL;


// TODO add to the class
double d2r(double v) {
	return v / 180 * M_PI;
}


// TODO add to the class
Eigen::Matrix4d frame_to_matrix(KDL::Frame frame)
{
	Eigen::Matrix4d matrix;
	for(int i=0; i < 4; i++)
	{
		for(int j=0; j < 4; j++)
		{
			matrix(i, j) = frame(i, j);
		}
	}
	return matrix;
}

// TODO add to the class
// TODO calc the pose from start and endpoints
Eigen::Matrix4d to_mat(KDL::Frame start_link, KDL::Frame end_link)
{
	Eigen::Matrix4d pose;
	Eigen::Matrix4d start_pose = frame_to_matrix(start_link);
	Eigen::Matrix4d end_pose = frame_to_matrix(end_link);
	Eigen::Vector4d origin(0, 0, 0, 1);
	Eigen::Vector4d basePoint = start_pose * origin;
	Eigen::Vector4d endPoint = end_pose * origin;


	return start_pose;
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
		std::cout << "\nnum_joints: " << nr_joints << std::endl;
	#endif //DEBUG

	// ---------------- initialise the arm to init point ------------- //

	// initailise the chain solver and the joint array
	ChainFkSolverPos_recursive fksolver = ChainFkSolverPos_recursive(chain);
	JntArray jointpositions = JntArray(nr_joints);

	// pass the joint angles from function input into the joint array
	for(int i=0; i<nr_joints; i++)
	{
		jointpositions(i) = 0.0;
	}


	// solve for the frame at the "link" of the chain for the given joint positions
	for(int link_num = 0; link_num < nr_joints; link_num++)
	{
		fksolver.JntToCart(jointpositions, *poses[link_num], link_num);
	}

	// TODO convert to config file
	double radii[6] = {0.04, 0.04, 0.04, 0.04, 0.04, 0.04};
	double lengths[6] = {0.15643, 0.12838, 0.21038, 0.21038, 0.20843, 0.10593};

	for(int i = 0; i < 6; i++)
	{
		Eigen::Matrix4d pose = to_mat(*poses[i], *poses[i+1]);
		Primitive* link = new Cylinder(pose, lengths[i], radii[i]);
		links.push_back(link);
	}

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
		if(fksolver.JntToCart(jointpositions, *poses[link_num], link_num) >= 0)
		{
			#ifdef DEBUG
			std::cout << "Calculations to link number: " << link_num << std::endl 
			          << *poses[link_num] << std::endl
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