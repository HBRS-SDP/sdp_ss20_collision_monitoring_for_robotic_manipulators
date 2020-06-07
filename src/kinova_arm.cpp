#include "kinova_arm.h"


// TODO add to the class
double d2r(double v) {
    return v / 180 * M_PI;
}


// TODO add to the class
Eigen::Matrix4d KinovaArm::frameToMatrix(KDL::Frame frame)
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
Eigen::Matrix4d KinovaArm::linkFramesToPose(KDL::Frame startLink, KDL::Frame endLink)
{
    Eigen::Matrix4d pose;
    Eigen::Matrix4d startPose = frameToMatrix(startLink);
    Eigen::Matrix4d endPose = frameToMatrix(endLink);
    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d basePoint = startPose * origin;
    Eigen::Vector4d endPoint = endPose * origin;
    Eigen::Matrix4d finalPose;

    if(!basePoint.isApprox(endPoint)) {
        Eigen::Vector3d midLine = (endPoint - basePoint).head<3>();
        Eigen::Vector3d directionVect(0, 0, 1);
        Eigen::Vector3d v = directionVect.cross(midLine);
        Eigen::Matrix3d r;
        // Check to see if the midline is only in the z direction
        if( v.isZero()) {
            double c = directionVect.dot(midLine);
            double s = v.norm();
            Eigen::Matrix3d k;
            k << 0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(2), 0;
            r = Eigen::MatrixXd::Identity(3,3) + k + (k*k)*((1-c)/(s*s));
        }
        else {
            r = Eigen::MatrixXd::Identity(3,3);
        }
        finalPose << r(0,0), r(0,1), r(0,2), basePoint(0),
                    r(1,0), r(1,1), r(1,2), basePoint(1),
                    r(2,0), r(2,1), r(2,2), basePoint(2),
                    0,      0,      0,      1;
    }
    else {
        finalPose << 0, 0, 0, 0,
                     0, 0, 0, 0,
                     0, 0, 0, 0,
                     0, 0, 0, 1;
    }


    return finalPose;
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
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    KDL::JntArray jointpositions = KDL::JntArray(nr_joints);

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
    radii.assign({0.04, 0.04, 0.04, 0.04, 0.04, 0.04});
    lengths.assign({0.15643, 0.12838, 0.21038, 0.21038, 0.20843, 0.10593});

    for(int i = 0; i < 6; i++)
    {
        Eigen::Matrix4d pose = linkFramesToPose(*poses[i], *poses[i+1]);
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
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    KDL::JntArray jointpositions = KDL::JntArray(nr_joints);

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
        if(link_num < nr_joints - 1)
        {
            links[link_num]->pose = linkFramesToPose(*poses[link_num], *poses[link_num+1]);
        }
    }

    return true;
}