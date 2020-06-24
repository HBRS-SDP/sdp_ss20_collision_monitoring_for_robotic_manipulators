// A simple program that computes the square root of a number
#include "arm_controller.h"
// #define DEBUG

ArmController::ArmController(Monitor* monitorObject, double k, double d,
                                                    double gamma, double beta) {
    Eigen::Matrix4d endpose;
    monitor = monitorObject;
    Eigen::Matrix4d currEndPose = monitor->arm->getPose();
    numJoints = monitor->arm->nJoints;

    
    this->obstaclePub = n.advertise<visualization_msgs::Marker>("kinova_controller/potential_field", 1000);

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
    #ifdef DEBUG
    std::cout << "goalCallback:\n\tCurrent goal: "<<this->goal<<std::endl;
    std::cout << "\tincoming goal: " << msg->x << ", " << msg->y << ", " << msg->z << std::endl;
    #endif // DEBUG
    this->goal[0] = msg->x;
    this->goal[1] = msg->y;
    this->goal[2] = msg->z;
    #ifdef DEBUG
    std::cout << "\tNew goal: "<<this->goal<<std::endl;
    #endif // DEBUG
}

Eigen::Vector3d ArmController::obstaclePotentialField(Eigen::Vector3d currentPosition, 
                                        Eigen::Vector3d velocity) {
    Eigen::Vector3d potentialField = Eigen::Vector3d({0, 0, 0});

    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector3d startArrow, positionLink;
    
    Capsule *endEffectorCapsule;
    
    double angle = 3.1415/2;

    for(int i = 0; i < monitor->obstacles.size(); i++) {
        Eigen::Vector3d direction; 
        monitor->arm->links.back()->getShortestDirection(direction, 
                                            monitor->obstacles[i]);

        endEffectorCapsule = dynamic_cast<Capsule*>(monitor->arm->links.back());
        if(endEffectorCapsule){
            startArrow = (monitor->obstacles[i]->pose * origin).head(3);
            //Shortest distance arrow (for visualization)
            MarkerPublisher mPublisherShortestDistance(obstaclePub, visualization_msgs::Marker::ARROW, "base_link", "shortest_distance", i, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0);
            
            mPublisherShortestDistance.setRadius(0.005);
            mPublisherShortestDistance.setLength(0.005);
            mPublisherShortestDistance.setPoints(startArrow, startArrow + direction - (direction / direction.norm()) * endEffectorCapsule->getRadius() );
            mPublisherShortestDistance.Publish();
        }

        if (velocity.norm() < 1.0e-2) {
            return potentialField;
        }
        
        
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
        rotation << ct + m_vt_0*rotvec(0), 
                    -m_st_2 +  m_vt_0_1,
                    m_st_1  +  m_vt_0_2,
                    m_st_2  +  m_vt_0_1,
                    ct      +  m_vt_1*rotvec(1),
                    -m_st_0 +  m_vt_1_2,
                    -m_st_1 +  m_vt_0_2,
                    m_st_0  +  m_vt_1_2,
                    ct      +  m_vt_2*rotvec(2);
        rotation.normalize();
        // Phi
        // φ = cos ((o − x) v/(|o − x| · |v|))
        double phi_denominator = velocity.norm()*direction.norm();
        double phi_numerator = direction.transpose()*velocity;
        double phi = std::acos(phi_numerator/phi_denominator);

        double exp = std::exp(-beta*phi);

        // R i vφ i exp(−βφ i )
        potentialField += (rotation * velocity) * phi * exp;


        #ifdef DEBUG
        std::cout << "direction: " << direction << std::endl;
        std::cout << "velocity: " << velocity << std::endl;
        std::cout << "rotvec: " << rotvec << std::endl;
        std::cout << "rotation: " << rotation << std::endl;
        std::cout << "phi_denominator" << phi_denominator << std::endl;
        std::cout << "phi_numerator" << phi_numerator << std::endl;
        std::cout << "phi" << phi << std::endl;
        std::cout << "exp" << exp << std::endl;

        std::cout << "potential field: " << potentialField << std::endl;
        #endif // DEBUG


        MarkerPublisher mPublisherPotentialField(obstaclePub, visualization_msgs::Marker::ARROW, "base_link", "potential_field", i, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0);
        
        mPublisherPotentialField.setRadius(0.005);
        mPublisherPotentialField.setLength(0.005);
        mPublisherPotentialField.setPoints(startArrow, startArrow + (gamma * potentialField));
        mPublisherPotentialField.Publish();
    }
   
    return gamma * potentialField;
}

KDL::Twist ArmController::controlLoop(void) {
    // Variable for storing the resulting joint velocities
    double x, y, z;
    // double alpha, beta, gamma;
    // Update the current state to match real arm state
    this->monitor->arm->updatePose(this->jointAngles);
    Eigen::Matrix4d currEndPose = monitor->arm->getPose();
    Eigen::Vector3d currEndPoint = (currEndPose * origin).head(3);
    objectDistances = monitor->distanceToObjects();
    armDistances = monitor->distanceBetweenArmLinks();

    //Show links as a cylinders
    Capsule *capsuleLink;
    Eigen::Vector3d positionLink;
    for(int i=0; i < monitor->arm->links.size(); i++){
        capsuleLink = dynamic_cast<Capsule*>(monitor->arm->links[i]);
        if(capsuleLink){
            double w = sqrt(1 + capsuleLink->pose(0,0)
                       + capsuleLink->pose(1,1) 
                       + capsuleLink->pose(2, 2))/2;
            double x = (capsuleLink->pose(2,1) - capsuleLink->pose(1, 2))/(4*w);
            double y = (capsuleLink->pose(0,2) - capsuleLink->pose(2, 0))/(4*w);
            double z = (capsuleLink->pose(1,0) - capsuleLink->pose(0, 1))/(4*w);

            Eigen::Quaterniond quat(x, y, z, w);

            Eigen::Vector4d basePoint(0, 0, capsuleLink->getLength() / 2.0 , 1);
            positionLink = (capsuleLink->pose * basePoint).head(3);
            
            MarkerPublisher mPublisherLink(obstaclePub, visualization_msgs::Marker::CYLINDER, "base_link", "links", i, positionLink(0), positionLink(1), positionLink(2), 0.0, 1.0, 0.0, 0.5);
            mPublisherLink.setRadius(capsuleLink->getRadius());
            mPublisherLink.setLength(capsuleLink->getLength());
            mPublisherLink.setOrientation(x, y, z, w);
            mPublisherLink.Publish();
        }
    }

    //v̇ = K(g−x) − Dv + p(x, v)
    Eigen::Vector3d currVelocity = {twist.vel.data[0], twist.vel.data[1], twist.vel.data[2]};
    Eigen::Vector3d newVelocity = K * (goal - currEndPoint) - D * currVelocity
                                + ArmController::obstaclePotentialField(currEndPoint, 
                                currVelocity);

    #ifdef DEBUG
    std::cout << "[ArmController] goal: " << goal << std::endl;
    std::cout << "[ArmController] currEndPoint: \n" << currEndPoint << std::endl;
    std::cout << "[ArmController] point to goal: \n" << goal - currEndPoint << std::endl;
    std::cout << "[ArmController] currVelocity: \n" << currVelocity << std::endl;
    std::cout << "[ArmController] newVelocity: \n" << newVelocity << std::endl;
    std::cout << "[ArmController] potential field: \n" << ArmController::obstaclePotentialField(currEndPoint, 
                                currVelocity) << std::endl;
    #endif // DEBUG

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
    #ifdef DEBUG

    #endif // DEBUG
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
    else if(msg->type == visualization_msgs::Marker::ARROW){
        #ifdef DEBUG
        std::cout << "arrow" << std::endl;
        #endif // DEBUG
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




