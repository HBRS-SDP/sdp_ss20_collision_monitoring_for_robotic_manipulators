#include "kinova_arm.h"


KinovaArm::KinovaArm(std::string urdf_filename){

    // ------------- import and initialise the KDL model --------------- //

    // Import the tree from urdf
    if (!kdl_parser::treeFromFile(urdf_filename, arm_tree)){
        std::cout << "Failed to construct kdl tree" << std::endl;
    }

    // Convert the tree to a chain and get the number of joints
    arm_tree.getChain("base_link", "EndEffector_Link", chain);
    nJoints = chain.getNrOfJoints();

    // init frames for all the joints
    for(int i = 0; i < nJoints; i++)
    {
        KDL::Frame* pose = new KDL::Frame();
        poses.push_back(pose);
    }
    
    #ifdef DEBUG
        std::cout << "\nnum_joints: " << nJoints << std::endl;
    #endif //DEBUG

    // ---------------- initialise the arm to init point ------------- //

    // initailise the chain solver and the joint array
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    KDL::JntArray jointArray = KDL::JntArray(nJoints);

    // pass the joint angles from function input into the joint array
    for(int i=0; i<nJoints; i++)
    {
        jointArray(i) = 0.0;
    }

    // solve for the frame at the "link" of the chain for the given joint positions
    for(int link_num = 0; link_num < nJoints; link_num++)
    {
        fksolver.JntToCart(jointArray, *poses[link_num], link_num);
    }

    // TODO convert to config file
    // pass in the parameters for the link cylinder models
    radii.assign({0.04, 0.04, 0.04, 0.04, 0.04, 0.04});
    lengths.assign({0.15643, 0.12838, 0.21038, 0.21038, 0.20843, 0.10593});

    // Create the new link objects in default position and 
    // add them to the links vector
    for(int i = 0; i < 6; i++)
    {
        Eigen::Matrix4d pose = linkFramesToPose(*poses[i], *poses[i+1]);
        Primitive* link = new Cylinder(pose, lengths[i], radii[i]);
        links.push_back(link);
    }

}

KinovaArm::~KinovaArm(){

    // delete the pose frames
    for(int i = 0; i < nJoints; i++)
    {
        delete(poses[i]);
    }

}

bool KinovaArm::updatePose(std::vector<double> jointPositions){

    // initailise the chain solver and the joint array
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(chain);
    KDL::JntArray jointArray= KDL::JntArray(nJoints);

    // pass the joint angles from function input into the joint array
    for(int i=0; i<nJoints; i++)
    {
        jointArray(i) = jointPositions[i];
    }


    // solve for the frame at the "link" of the chain for the given joint positions
    for(int link_num = 0; link_num < nJoints; link_num++)
    {
        if(fksolver.JntToCart(jointArray, *poses[link_num], link_num) >= 0)
        {
            #ifdef DEBUG
            // print the resulting link calculations
            std::cout << "Calculations to link number: " << link_num << std::endl 
                      << *poses[link_num] << std::endl
                      << "Success" << std::endl;
            #endif //DEBUG
        }
        // If calculation fails print error and return false
        else
        {
            std::cout << "Error: could not calculate forward kinematics" << std::endl;
            return false;
        }

        // For all the link objects (nJoints-1) update the pose.
        if(link_num != 0)
        {
            links[link_num-1]->pose = linkFramesToPose(*poses[link_num-1], *poses[link_num]);
        }
    }

    // Return true if performed successfully
    return true;
}


Eigen::Matrix4d KinovaArm::frameToMatrix(KDL::Frame frame)
{
    // init a matrix to fill
    Eigen::Matrix4d matrix;

    // Copy the values across index by index
    for(int i=0; i < 4; i++)
    {
        for(int j=0; j < 4; j++)
        {
            matrix(i, j) = frame(i, j);
        }
    }

    // return the eigen matrix
    return matrix;
}


Eigen::Matrix4d KinovaArm::linkFramesToPose(KDL::Frame startLink, KDL::Frame endLink)
{
    // Convert the frames to eigen matrices for the maths
    Eigen::Matrix4d startPose = frameToMatrix(startLink);
    Eigen::Matrix4d endPose = frameToMatrix(endLink);

    // Create origin matrix to get start and end points
    Eigen::Vector4d origin(0, 0, 0, 1);

    // Find the start and end points
    Eigen::Vector4d basePoint = startPose * origin;
    Eigen::Vector4d endPoint = endPose * origin;

    // create matrix to store the final pose to be returned
    Eigen::Matrix4d finalPose;
    
    // Check to see if the link a starts and originates at the same point
    if(fabs((basePoint - endPoint).norm()) > 0.0001) {

        // Get the vector representing the line from the start to end point
        Eigen::Vector3d midLine = (endPoint - basePoint).head(3);
        // Create a vector in the z direction
        Eigen::Vector3d directionVect(0, 0, 1);
        // Get the vector that is prependicular to the midline and z vector
        Eigen::Vector3d v = directionVect.cross(midLine);
        // matrix to store the rotaion matrix
        Eigen::Matrix3d r;

        // Check to see if the midline is only in the z direction
        if( !v.isZero()) {
            // Build the rotation vector
            double c = directionVect.dot(midLine);
            double s = v.norm();
            Eigen::Matrix3d k;
            k << 0, -v(2), v(1),
                v(2), 0, -v(0),
                -v(1), v(2), 0;
            r = Eigen::MatrixXd::Identity(3,3) + k + (k*k)*((1-c)/(s*s));
        }
        else {
            // no rotation required, return identity matrix
            r = Eigen::MatrixXd::Identity(3,3);
        }

        // Construct the final matrix based off the start point and rotation Mat
        finalPose << r(0,0), r(0,1), r(0,2), basePoint(0),
                    r(1,0), r(1,1), r(1,2), basePoint(1),
                    r(2,0), r(2,1), r(2,2), basePoint(2),
                    0,      0,      0,      1;
    }
    else {
        // If they are too close together return a 0 matrix
        finalPose << 0, 0, 0, 0,
                     0, 0, 0, 0,
                     0, 0, 0, 0,
                     0, 0, 0, 1;
    }

    // Return the final pose
    return finalPose;
}

