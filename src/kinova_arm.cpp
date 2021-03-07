#include "kinova_arm.h"
#define MAX_JOINT_VEL 10

// #define DEBUG


KinovaArm::KinovaArm(std::string urdf_filename){

    // ------------- import and initialise the KDL model --------------- //
    /// This is used to import the URDF file
    KDL::Tree armTree;

    // Import the tree from urdf
    if (!kdl_parser::treeFromFile(urdf_filename, armTree)){
        std::cout << "[KinovaArm] Failed to construct kdl tree" << std::endl;
    }
    // Convert the tree to a chain and get the number of joints
    armTree.getChain("base_link", "EndEffector_Link", fkChain);
    nJoints = fkChain.getNrOfJoints();
    nLinks = fkChain.getNrOfSegments();
    nFrames = nLinks+1;

    // init frames for all the joints
    for(int i = 0; i < nFrames; i++)
    {
        localPoses.push_back(new KDL::Frame());
    }
    #ifdef DEBUG
        std::cout << "\nnum_joints: " << nJoints << " " << localPoses.size() << std::endl;
        std::cout << "fkChain.getNrOfSegments(): " << fkChain.getNrOfSegments() << std::endl;
        std::cout << "fkChain.getNrOfJoints(): " << fkChain.getNrOfJoints() << std::endl;
        for(int i =0; i < nLinks ; i++) {
            std::cout << fkChain.getSegment(i).getName()<< ":\n"<<fkChain.getSegment(i).getFrameToTip() << std::endl;
        }
    #endif //DEBUG

    // ---------------- initialise the arm to init point ------------- //

    // initailise the chain solver and the joint array
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(fkChain);
    jointArray = KDL::JntArray(nJoints);
    jointVels = KDL::JntArray(nJoints);

    // pass the joint angles from function input into the joint array
    for(int i=0; i<nJoints; i++)
    {
        jointArray(i) = M_PI_2;
        jointVels(i) = 0.0;
    }

    // solve for the frame at the "link" of the chain for the given joint positions
    for(int frameNum = 0; frameNum < nFrames; frameNum++)
    {
        fksolver.JntToCart(jointArray, *localPoses[frameNum], frameNum);
    }

    // pass in the parameters for the link cylinder models
    radii.assign({0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04});
    lengths.assign({0.15643, 0.12838, 0.21038, 0.21038, 0.20843, 0.10593, 0.10593, 0.061525});

    // Create the new link objects in default position and 
    // add them to the links vector
    for(int linkNum = 0; linkNum < nLinks; linkNum++)
    {
        Eigen::Matrix4d pose = linkFramesToPose(*localPoses[linkNum], *localPoses[linkNum+1]);
        Capsule* link = new Capsule(pose, lengths[linkNum], radii[linkNum]);
        links.push_back(link);
    }
    #ifdef DEBUG
    std::cout << "links.size(): " << links.size() << std::endl;
    #endif
    // Mathematical constants, declared in constructor for speed
    this->baseTransform << 1, 0, 0, 0,
                           0, 1, 0, 0,
                           0, 0, 1, 0,
                           0, 0, 0, 1;
    this->origin << 0, 0, 0, 1;
    this->directionVect << 0, 0, 1;
    this->i3 << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
}

NarkinBase::NarkinBase(Eigen::Vector3d inputBaseTransform){
    this->baseTransform = inputBaseTransform;

    // localPose =new KDL::Frame();
    Eigen::Vector3d pose = inputBaseTransform;
    double x= 0.66;
    double y= 0.60;
    double z= 0.30;
    
    // ======================= need to import pose of narkin base from urdf model          
    //Eigen::Matrix4d pose = linkFramesToPose(*localPoses[linkNum], *localPoses[linkNum+1]);
    Box3* base = new Box3(pose, x, y, z);
    this->base_primitive =base;


}
KinovaArm::KinovaArm(std::string urdf_filename, Eigen::Matrix4d inputBaseTransform){

    // ------------- import and initialise the KDL model --------------- //
    /// This is used to import the URDF file
    KDL::Tree armTree;

    this->baseTransform = inputBaseTransform;

    // Import the tree from urdf
    if (!kdl_parser::treeFromFile(urdf_filename, armTree)){
        std::cout << "[KinovaArm] Failed to construct kdl tree" << std::endl;
    }
    // Convert the tree to a chain and get the number of joints
    armTree.getChain("base_link", "EndEffector_Link", fkChain);
    nJoints = fkChain.getNrOfJoints();
    nLinks = fkChain.getNrOfSegments();
    nFrames = nLinks+1;

    // init frames for all the joints
    for(int i = 0; i < nFrames; i++)
    {
        localPoses.push_back(new KDL::Frame());
    }
    #ifdef DEBUG
        std::cout << "[KinovaArm] num_joints: " << nJoints << " " << localPoses.size() << std::endl;
    #endif //DEBUG

    // ---------------- initialise the arm to init point ------------- //

    // initailise the chain solver and the joint array
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(fkChain);
    jointArray = KDL::JntArray(nJoints);
    jointVels = KDL::JntArray(nJoints);

    // pass the joint angles from function input into the joint array
    for(int i=0; i<nJoints; i++)
    {
        jointArray(i) = M_PI_2;
        jointVels(i) = 0.0;
    }

    // solve for the frame at the "link" of the chain for the given joint positions
    for(int frameNum = 0; frameNum < nFrames; frameNum++)
    {
        fksolver.JntToCart(jointArray, *localPoses[frameNum], frameNum);
    }

    // pass in the parameters for the link cylinder models
    radii.assign({0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04});
    lengths.assign({0.15643, 0.12838, 0.21038, 0.21038, 0.20843, 0.10593, 0.10593, 0.061525});

    // Create the new link objects in default position and 
    // add them to the links vector
    for(int linkNum = 0; linkNum < nLinks; linkNum++)
    {
        Eigen::Matrix4d pose = linkFramesToPose(*localPoses[linkNum], *localPoses[linkNum+1]);
        Capsule* link = new Capsule(pose, lengths[linkNum], radii[linkNum]);
        links.push_back(link);
    }
    #ifdef DEBUG
    std::cout << "# links: " << links.size() << std::endl;
    #endif
    // Mathematical constants, declared in constructor for speed
    this->origin << 0, 0, 0, 1;
    this->directionVect << 0, 0, 1;
    this->i3 << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
}

KinovaArm::~KinovaArm(){

    for(int i=0; i < links.size(); i++){
        delete(links[i]);
    }

    for(int i=0; i < localPoses.size(); i++){
        delete(localPoses[i]);
    }
}


NarkinBase::~NarkinBase(){

    
    delete(base_primitive);

    delete(localPose);
    
}

bool NarkinBase::updatePose(Eigen::Vector3d basePositions){
    this->baseTransform = basePositions;
    // this->base_primitive->box_center = basePositions;
    return true;
}

bool KinovaArm::updatePose(std::vector<double> jointPositions){

    // initailise the chain solver and the joint array
    KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(fkChain);

    // pass the joint angles from function input into the joint array
    for(int i=0; i<nJoints; i++)
    {
        jointArray(i) = jointPositions[i];
    }
    

    // solve for the frame at the "link" of the chain for the given joint positions
    for(int frameNum = 0; frameNum < nFrames; frameNum++)
    {
        if(fksolver.JntToCart(jointArray, *localPoses[frameNum], frameNum) >= 0)
        {
            #ifdef DEBUG
            // print the resulting link calculations
            std::cout << "Calculations to link number: " << frameNum << std::endl 
                      << *localPoses[frameNum] << std::endl
                      << "Success" << std::endl;
            #endif //DEBUG
        }
        // If calculation fails print error and return false
        else
        {
            std::cout << "Error: could not calculate forward kinematics" << std::endl;
            return false;
        }

        // For all the link objects (nFrames-1) update the pose.
        if(frameNum != 0)
        {
            links[frameNum-1]->pose = linkFramesToPose(*localPoses[frameNum-1], *localPoses[frameNum]);
        }
    }

    // Return true if performed successfully
    return true;
}


std::vector<double> KinovaArm::ikVelocitySolver(KDL::Twist twist){

    // vector to store output values
    std::vector<double> jointVelocitiesOut;

    // inverse kinemetics solver
    KDL::ChainIkSolverVel_wdls ikSolver = KDL::ChainIkSolverVel_wdls(fkChain);

    // Set the weights for singularity handling
    Eigen::MatrixXd weight_ts;
    weight_ts.resize(6, 6);
    weight_ts.setIdentity();
    weight_ts(0, 0) = 1;
    weight_ts(1, 1) = 1;
    weight_ts(2, 2) = 1;
    weight_ts(3, 3) = 0.4;
    weight_ts(4, 4) = 0.4;
    weight_ts(5, 5) = 0.4;
    ikSolver.setWeightTS(weight_ts);

    //solver for joint velocities
    ikSolver.CartToJnt(jointArray, twist, jointVels);

    // push velocities onto the vector
    for (int i=0; i<jointVels.rows(); i++){
        double velocity;

        // Keep the max velocity to the limit
        if(jointVels(i) > MAX_JOINT_VEL) {
            velocity = MAX_JOINT_VEL;
        }
        else if(jointVels(i) < -MAX_JOINT_VEL) {
            velocity = -MAX_JOINT_VEL;
        }
        else {
            velocity = jointVels(i);
        }

        // Add the joint angles to the list
        jointVelocitiesOut.push_back(velocity);
    }

    // Return the joint velocities
    return jointVelocitiesOut;
}

Eigen::Vector3d NarkinBase::getPose(void){

   //must return latest pose from frames
    
    
    // init a matrix to fill
    // Eigen::Matrix4d matrix;

    // // Copy the values across index by index
    // for(int i=0; i < 4; i++)
    // {
    //     for(int j=0; j < 4; j++)
    //     {
    //         matrix(i, j) = frame(i, j);
    //     }
    // }

    // // return the eigen matrix
    // return baseTransform * matrix;

	return this->baseTransform;



}

Eigen::Matrix4d KinovaArm::getPose(void)
{
    // Get the final pose from the list
    return frameToMatrix(*localPoses.back());
}

Eigen::Matrix4d KinovaArm::getPose(int frameNumber)
{
    // input sanitization
    if(frameNumber >= localPoses.size() | frameNumber < 0){
        std::cout << "Access joint number larger than array in getPose.\n"<<
                     "Returning endeffector Pose"<<std::endl;
        return frameToMatrix(*localPoses.back());
    }

    // return joint pose
    return frameToMatrix(*localPoses[frameNumber]);
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
    return baseTransform * matrix;
}


Eigen::Matrix4d KinovaArm::linkFramesToPose(KDL::Frame startLink, KDL::Frame endLink)
{
    // Convert the frames to eigen matrices for the maths
    Eigen::Matrix4d startPose = frameToMatrix(startLink);
    Eigen::Matrix4d endPose = frameToMatrix(endLink);

    // Find the start and end points
    Eigen::Vector4d basePoint = startPose * origin;
    Eigen::Vector4d endPoint = endPose * origin;

    // create matrix to store the final pose to be returned
    Eigen::Matrix4d finalPose;
    
    // Check to see if the link a starts and originates at the same point
    if(fabs((basePoint - endPoint).norm()) > 0.0001) {
        // Get the vector representing the line from the start to end point
        Eigen::Vector3d midLine = (endPoint - basePoint).head(3);
        midLine = midLine.normalized();
        // Get the vector that is prependicular to the midline and z vector
        // matrix to store the rotaion matrix
        Eigen::Matrix3d r = i3;
        // Check to see if the midline is only in the z direction
        
        if(midLine(2) != 0) {
            r << 1, 0,            midLine(0),
                 0, 1/midLine(2), midLine(1),
                 0, 0,            midLine(2);
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

