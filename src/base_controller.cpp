
#include "base_controller.h"


 BaseController::BaseController(Monitor* monitorObject, double k, double d, 
                                            double gamma, double beta){
  
    // Intialise the controller based off the monitor
    Eigen::Vector3d currBasePose;
    this->monitor = monitorObject;
    currBasePose = monitor->base->getPose();
    

    // Constants for obstacle avoidance
    this->K = k;
    this->D = d;
    this->gamma = gamma;
    this->beta = beta;

    // PID constants should be declared here
    //P= ;
    //I= ;
    //D= ;


  speed.linear.x=0.0;
  speed.linear.y=0.0;
  speed.linear.z=0.0;
  speed.angular.x=0.0;
  speed.angular.y=0.0;
  speed.angular.z=0.0;



     
    // ======== need to describe these topics to be subscribed these are for potnetial fields

    // this->arrowsPub = n.advertise<visualization_msgs::Marker>("kinova_controller/distance_field", 1000);
    // this->linksCylindersPub = n.advertise<visualization_msgs::Marker>("kinova_controller/links_cylinders", 1000);
    // this->linksSpheresPub = n.advertise<visualization_msgs::Marker>("kinova_controller/links_spheres", 1000);

    this->arrowsPub_base = n.advertise<visualization_msgs::Marker>("base_controller/distance_field_base", 1000);
    this->CubePub = n.advertise<visualization_msgs::Marker>("/Narko_base_marker_cpp", 1000);

    origin << 0, 0, 0, 1;
    this->goal = currBasePose;// * origin).head(3);
    objectDistances = monitor->baseDistanceToObjects();

}

BaseController::~BaseController() {
    // Delete all the rviz obstacles
    for (int i=0; i<rvizObstacles.size();i++) {
        delete(rvizObstacles[i]);
    }

    // Delete all the obstacles allocated at runtime
    for (int i=0; i<obstaclesAllocated.size();i++) {
        delete(obstaclesAllocated[i]);
    }
}

void BaseController::baseCallback(const nav_msgs::Odometry::ConstPtr &msg)
      {
         
    tf2::Quaternion q_orig, q_rot, q_new;

        basePosition[0]=msg->pose.pose.position.x;
        basePosition[1]=msg->pose.pose.position.y;
     
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
     
    tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
       
    basePosition[2]= yaw; 
 
}
     
     
void BaseController::goalCallback(const geometry_msgs::Point::ConstPtr& msg){

      //transform the message from its current type to Eigen::Vector3d and put in goal variable
    this->goal[0] = msg->x;
    this->goal[1] = msg->y;
    this->goal[2] = msg->z;

   

}


void BaseController::updateObstacles(const visualization_msgs::Marker::ConstPtr& msg){

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
    else if(msg->type == visualization_msgs::Marker::CYLINDER){
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
            rvizObstacle->pose(2, 3) -= (rvizObstacle->marker.scale.z / 2.0);
            Capsule* capsule = new Capsule(rvizObstacle->pose, rvizObstacle->marker.scale.z, rvizObstacle->marker.scale.x / 2);
            obstaclesAllocated.push_back(capsule);
            monitor->addObstacle(capsule);
        }
    }
// ==================== Box obstacle ======================================

 else if(msg->type == visualization_msgs::Marker::CUBE){
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
            Box3* box = new Box3(rvizObstacle->pose, rvizObstacle->marker.scale.x/2,rvizObstacle->marker.scale.y/2, rvizObstacle->marker.scale.z/2);
            obstaclesAllocated.push_back(box);
            monitor->addObstacle(box);
        }
    }


    
//========================================================================

    else {
       // ROS_ERROR("Wrong shape for obstacle");
    }

}




Eigen::Vector3d BaseController::obstaclePotentialField(Eigen::Vector3d currentPosition, 
                                        Eigen::Vector3d velocity) {
       // init the variables used in the maths
    Eigen::Vector3d potentialField = Eigen::Vector3d({0, 0, 0});
    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector3d startArrow, positionLink;
    Eigen::Vector3d ownClosestPoint, obstacleClosestPoint;
     //ROS_ERROR_STREAM(" field function called: \n ");  
     Box3 *baseCube;
     double angle = 3.1415/2;
     

     if (monitor->obstacles.empty()!=0){
      std::vector<Primitive*> obstacles; 
      obstacles.reserve(monitor->obstacles.size());
      obstacles.insert(obstacles.end(), monitor->obstacles.begin(), monitor->obstacles.end());
    //   obstacles.pop_back();
     
      for(int i = 0; i < obstacles.size(); i++) {
        Eigen::Vector3d direction;
         //ROS_ERROR_STREAM(" field function called in loop: \n "<< typeid(obstacles[i]).name()); 

   
              
         // ROS_ERROR_STREAM(" field function called before shortest distance : \n "); 
        
        monitor->base->base_primitive->getShortestDirection(direction, 
                                            obstacles[i]);


        baseCube = dynamic_cast<Box3*>(monitor->base->base_primitive);  

        if(baseCube){

            //startArrow = (obstacles[i]->pose * origin).head(3);

            Eigen::MatrixXd closestPoints(2, 3);
            
            baseCube->getClosestPoints(closestPoints, obstacles[i]);
             
            ownClosestPoint = closestPoints.row(0);
            obstacleClosestPoint = closestPoints.row(1);
            Eigen::Vector3d h={1,1,0};
             obstacleClosestPoint=h;



            startArrow = baseCube->box_center; // currently arrow starting from center of cube , needs to change to own closest point 
         //=== need to chamge it to narko base topic   
            MarkerPublisher mPublisherShortestDistance(arrowsPub_base, visualization_msgs::Marker::ARROW, "Narko_base_link", "shortest_distance_base_arrow", i, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0); 
            mPublisherShortestDistance.setRadius(0.005);
            mPublisherShortestDistance.setLength(0.005);
            mPublisherShortestDistance.setPoints(obstacleClosestPoint, ownClosestPoint);// + (direction / direction.norm()) * endEffectorCapsule->getRadius() );
            mPublisherShortestDistance.Publish();
            }

         // ROS_ERROR_STREAM(" basecontroller reached here: \n ");   
        
        // Eigen::Vector3d retFiled = {0.1,0.1,0.1};

        //  return retFiled; 

        if (velocity.norm() < 1.0e-2) {
            return potentialField;
           // ROS_WARN_STREAM("PField in basecontroller inside if: \n " << potentialField<<"\n\n");
        } 
        
 
        // Rotation matrix
        Eigen::Vector3d rotvec = direction.cross(velocity);
        rotvec.normalized();

        // Build the rotation matrix from the rodreges vector
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

        //Perform the related maths from [1]
        rotation.normalize();
       
       
       
        // Phi
        // φ = cos ((o − x) v/(|o − x| · |v|))
        // double phi_denominator = velocity.norm()*direction.norm();

        Eigen::Vector3d velocity_abs = velocity.cwiseAbs();
        Eigen::Vector3d direction_abs = direction.cwiseAbs();

        double phi_denominator= velocity_abs.dot(direction_abs); 
        // double phi_denominator =direction_abs.adjoint()* velocity_abs;

         
       double phi_numerator = direction.dot(velocity);
        // double phi_numerator = direction.transpose()*velocity;
        double phi = std::acos(phi_numerator/phi_denominator);

        double exp = std::exp(-beta*phi);

        // R i vφ i exp(−βφ i )
        potentialField += (rotation * velocity) * phi * exp;
       // ROS_WARN_STREAM("PField in basecontroller: \n " << potentialField<<"\n\n");
   
        MarkerPublisher mPublisherPotentialField(arrowsPub_base, visualization_msgs::Marker::ARROW, "base_link_arrow", "potential_field_base", i, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0);
        
        mPublisherPotentialField.setRadius(0.005);
        mPublisherPotentialField.setLength(0.005);
        mPublisherPotentialField.setPoints(obstacleClosestPoint, obstacleClosestPoint + (gamma * potentialField));
        mPublisherPotentialField.Publish();
    
        
        }

        return gamma * potentialField;

     }

     else {   Eigen::Vector3d zeroField ={0,0,0};
            
         return  zeroField;}
    }

geometry_msgs::Twist BaseController:: control_loop(){
    // Variable for storing the resulting joint velocities
    double x, y, z;
    // Update the current state to match real base state
    this->monitor->base->updatePose(this->basePosition);

    // Eigen::Matrix4d currBasePose = monitor->base->getPose();
    Eigen::Vector3d currBasePoint =  monitor->base->getPose();;
    objectDistances = monitor->baseDistanceToObjects();
    // ROS_WARN_STREAM(objectDistances[0]);
    // Show base as box

    Box3 *cubebase;
   //======= This end point needs to be changes to point on obstacle 
    Eigen::Vector3d end_point(0.0,0.5,0.1);
    Eigen::Vector3d positionLink, startArrow, endArrow;
    cubebase = dynamic_cast<Box3*>(monitor->base);
    if (cubebase){

        startArrow = cubebase->box_center;
        endArrow = (cubebase->box_center + end_point ).head(3);
        geometry_msgs::Vector3 scale;
        float array[2];// = new float;
        array[0]= cubebase->extents[0]/2;
        array[1]= cubebase->extents[0]/2;
        array[2]= cubebase->extents[0]/2;
        
        // scale[1] =cubebase->extents[1]/2;
        // scale[2]=cubebase->extents[2]/2;
        
            MarkerPublisher mPublisherLink(CubePub, visualization_msgs::Marker::CUBE, "narko_base_link_marker_cpp", "base _link_cpp", 20, positionLink(0), positionLink(1), positionLink(2), 0.0, 0.0, 1.0, 0.5);
            mPublisherLink.setScale(scale);                       
            mPublisherLink.Publish();

    }
  
  geometry_msgs::Twist speed;

double inc_x= goal[0] - basePosition[0];
double  inc_y= goal[1] - basePosition[1] ;
auto th_to_follow = atan2( inc_y,inc_x ) ; 
double  inc_th= th_to_follow-basePosition[2];


//   speed.linear.x=inc_x*0.1;
//   speed.linear.y=inc_y*0.1;
//   speed.angular.z=inc_th*0.1;

     





Eigen::Vector3d destination = {inc_x, inc_y, inc_th};

Eigen::Vector3d my_field = {0.1, 0.2, 0.4};


            // calculating difference between current and goal orientation of robot base  
            //    auto th_to_follow = atan2( inc_y,inc_x ) ;          

       
                
  Eigen::Vector3d currVelocity = {speed.linear.x, speed.linear.y, speed.angular.z};
   Eigen::Vector3d newVelocity = K * (destination) - D * currVelocity
                                + BaseController::obstaclePotentialField(currBasePoint, 
                                currVelocity);



// Eigen::Vector3d currVelocity = {speed.linear.x, speed.linear.y, speed.angular.z};
//    Eigen::Vector3d newVelocity = K * (destination) - D * currVelocity
//                                 + my_field ;


     
    Eigen::Vector4d transformedVel = {newVelocity[0], newVelocity[1], newVelocity[2], 0.0};
    // transformedVel = monitor->arm->getPose() * transformedVel;
    x = newVelocity[0];
    y = newVelocity[1];
    z = newVelocity[2];

    speed.linear.x = x;
    speed.linear.y = y;
    speed.angular.z = z; 

    return speed; 


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




