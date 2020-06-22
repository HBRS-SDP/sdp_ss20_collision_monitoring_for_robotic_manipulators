// A simple program that computes the square root of a number
#include "arm_controller.h"

ArmController::ArmController(Monitor* monitorObject, double k, double d,
                                                    double gamma, double beta) {
    Eigen::Matrix4d endpose;
    monitor = monitorObject;
    Eigen::Matrix4d currEndPose = monitor->arm->getPose();
    numJoints = monitor->arm->nJoints;

    origin << 0, 0, 0, 1;

    goal = (currEndPose * origin).head(3);
    objectDistances = monitor->distanceToObjects();
    armDistances = monitor->distanceBetweenArmLinks();

    for(int i=0; i<numJoints; i++) {
        jointAngles.push_back(0.0);
    }

    // Constants for obstacle avoidance
    K = k;
    D = d;
    this->gamma = gamma;
    this->beta = beta;

    // Initiate current velocity as 0
    twist.vel = KDL::Vector {0, 0, 0 };
    twist.rot = KDL::Vector {0, 0, 0 };

}

ArmController::~ArmController() {
    for (int i=0; i<rvizObstacles.size();i++) {
        delete(rvizObstacles[i]);
    }
    for (int i=0; i<obstaclesAllocated.size();i++) {
        delete(obstaclesAllocated[i]);
    }
}

void ArmController::armCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    jointAngles.assign(msg->position.begin(), msg->position.end());

}

void ArmController::goalCallback(const geometry_msgs::Point::ConstPtr& msg) {
    //transform the message from its current type to Eigen::Vector3d and put in goal variable
    std::cout << "goalCallback:\n\tCurrent goal: "<<this->goal<<std::endl;
    std::cout << "\tincoming goal: " << msg->x << ", " << msg->y << ", " << msg->z << std::endl;
    this->goal[0] = msg->x;
    this->goal[1] = msg->y;
    this->goal[2] = msg->z;
    std::cout << "\tNew goal: "<<this->goal<<std::endl;
}

Eigen::Vector3d ArmController::obstaclePotentialField(Eigen::Vector3d currentPosition, 
                                        Eigen::Vector3d velocity) {
    Eigen::Vector3d potentialField = Eigen::Vector3d({0, 0, 0});
    double angle = 3.1415/2;
    
    for(int i = 0; i < monitor->obstacles.size(); i++) {
        
        Eigen::Vector3d direction; 
        monitor->arm->links.back()->getShortestDirection(direction, 
                                            monitor->obstacles[i]);
        // Rotation matrix
        Eigen::Vector3d rotvec = direction.cross(velocity);
        rotvec.normalized();

        double ct = cos(angle);
        double st = sin(angle);
        double vt = 1-ct;
        double m_vt_0=vt*rotvec(0);
        double m_vt_1=vt*rotvec(1);
        double m_vt_2=vt*rotvec(2);
        double m_st_0=rotvec(0)*st;
        double m_st_1=rotvec(1)*st;
        double m_st_2=rotvec(2)*st;
        double m_vt_0_1=m_vt_0*rotvec(1);
        double m_vt_0_2=m_vt_0*rotvec(2);
        double m_vt_1_2=m_vt_1*rotvec(2);

        Eigen::Matrix3d rotation;
        rotation << ct      + m_vt_0*rotvec(0), 
                    -m_st_2 +  m_vt_0_1,
                    m_st_1  +  m_vt_0_2,
                    m_st_2  +  m_vt_0_1,
                    ct      +  m_v  t_1*rotvec(1),
                    -m_st_0 +  m_vt_1_2,
                    -m_st_1 +  m_vt_0_2,
                    m_st_0  +  m_vt_1_2,
                    ct      +  m_vt_2*rotvec(2);

        // Phi
        // φ = cos ((o − x) v/(|o − x| · |v|))
        double phi_denominator = velocity.norm()*direction.norm();
        double phi_numerator = direction.transpose()*velocity;
        double phi = std::acos(phi_numerator/phi_denominator);

        double exp = std::exp(-beta*phi);

        // R i vφ i exp(−βφ i )
        potentialField += (rotation * velocity) * phi * exp;
    }
    return gamma * potentialField;
}

KDL::Twist ArmController::controlLoop(void) {
    // Variable for storing the resulting joint velocities
    double x, y, z, alpha, beta, gamma;
    // Update the current state to match real arm state
    this->monitor->arm->updatePose(this->jointAngles);
    Eigen::Matrix4d currEndPose = monitor->arm->getPose();
    Eigen::Vector3d currEndPoint = (currEndPose * origin).head(3);
    objectDistances = monitor->distanceToObjects();
    armDistances = monitor->distanceBetweenArmLinks();

    //v̇ = K(g−x) − Dv + p(x, v)

    Eigen::Vector3d currVelocity = {twist.vel.data[0], twist.vel.data[1], twist.vel.data[2]};
    Eigen::Vector3d newVelocity = K * (goal - currEndPoint) - D * currVelocity
                                + ArmController::obstaclePotentialField(currEndPoint, 
                                currVelocity);

    std::cout << newVelocity << std::endl;

    std::cout << "goal: " << goal << std::endl;
    std::cout << "currEndPoint: \n" << currEndPoint << std::endl;
    std::cout << "point to goal: \n" << goal - currEndPoint << std::endl;
    std::cout << "currVelocity: \n" << currVelocity << std::endl;
    std::cout << "newVelocity: \n" << newVelocity << std::endl;
    std::cout << "potential feild: \n" << ArmController::obstaclePotentialField(currEndPoint, 
                                currVelocity) << std::endl;

    Eigen::Vector4d transformedVel = {newVelocity[0], newVelocity[1], newVelocity[2], 0.0};
    // transformedVel = monitor->arm->getPose() * transformedVel;
    x = newVelocity[0];
    y = newVelocity[1];
    z = newVelocity[2];

    twist.vel = KDL::Vector { x, y, z };
    twist.rot = KDL::Vector {0, 0, 0 };
    return twist;
}

void ArmController::updateObstacles(const visualization_msgs::Marker::ConstPtr& msg) {
    std::cout << "New obstacle of type: " << msg->type <<std::endl;
    if (msg->type == visualization_msgs::Marker::SPHERE) {
        bool newObstacle = true;
        for(int i=0; i<rvizObstacles.size();i++){
            if(rvizObstacles[i]->marker.id == msg->id) {
                newObstacle = false;
                int index = rvizObstacles[i]->idx;
                monitor->obstacles[index]->pose = rvizObstacles[i]->updatePose(msg);
            }
        }

        if(newObstacle) {
            RvizObstacle* rvizObstacle = new RvizObstacle(msg, rvizObstacles.size());
            rvizObstacles.push_back(rvizObstacle);
            Sphere* sphere = new Sphere(rvizObstacle->pose, rvizObstacle->marker.scale.x);
            obstaclesAllocated.push_back(sphere);
            monitor->addObstacle(sphere);
        }
    }
    else {
        ROS_ERROR("Wrong shape for obstacle");
    }
}


RvizObstacle::RvizObstacle(visualization_msgs::Marker::ConstPtr markerIn, int index) {
    marker = *markerIn;
    idx = index;
    double qx = marker.pose.orientation.x;
    double qy = marker.pose.orientation.y;
    double qz = marker.pose.orientation.z;
    double qw = marker.pose.orientation.w;
    double px = marker.pose.position.x;
    double py = marker.pose.position.y;
    double pz = marker.pose.position.z;
    pose << 1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw, px,
            2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw, py,
            2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy, pz,
            0, 0, 0, 1;
}

RvizObstacle::~RvizObstacle() {
}

Eigen::Matrix4d RvizObstacle::updatePose(visualization_msgs::Marker::ConstPtr markerIn) {
    marker = *markerIn;
    double qx = marker.pose.orientation.x;
    double qy = marker.pose.orientation.y;
    double qz = marker.pose.orientation.z;
    double qw = marker.pose.orientation.w;
    double px = marker.pose.position.x;
    double py = marker.pose.position.y;
    double pz = marker.pose.position.z;
    pose << 1-2*qy*qy-2*qz*qz, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw, px,
            2*qx*qy+2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz-2*qx*qw, py,
            2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx*qx-2*qy*qy, pz,
            0, 0, 0, 1;
    return pose;
}




