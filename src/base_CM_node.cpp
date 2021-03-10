// A simple program that computes the square root of a number
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <kdl/chain.hpp>
#include "primitives.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
// #include "arm_controller.h"
#include "base_controller.h"
#include "sensor_msgs/JointState.h"
#include "visualization_msgs/Marker.h"
#include "marker_publisher.h"

int main(int argc, char **argv){

   ros::init(argc, argv, "base_controller");
    std::string nodeName = ros::this_node::getName();
    #ifdef DEBUG
        std::cout << "nodeName: " << nodeName << std::endl;
    #endif //DEBUG
    ros::NodeHandle n;
    std::string goalTopic;
    std::string narkoPosition;

// setup the first monitor function
    n.param<std::string>(ros::this_node::getName()+"/goal_topic", goalTopic, "/goal_point");
    n.param<std::string>(ros::this_node::getName()+"/narko_base_link", narkoPosition, "/narko_base_link");
    double K;
    double D;
    double gamma;
    double beta;
    n.param<double>("/K", K, 0.5);
    n.param<double>("/D", D, 0);
    n.param<double>("/gamma", gamma, 100);
    n.param<double>("/beta", beta, 20/3.1425);

//    Eigen::Matrix4d basePosition;
//    basePosition<< 0,0,0,0.0,
//                   0,0,0,0.0,
//                   0,0,0,0.0,
//                   0,0,0,1;


  Eigen::Vector3d basePosition;

  basePosition[0]=0.0;
  basePosition[1]=0.0;
  basePosition[2]=-1.5;
 
    NarkinBase base1(basePosition);
    // KinovaArm arm1(model);
    Monitor monitor1(&base1);
    // double radius_1 = 30;
    // Eigen::Matrix4d pose_1;
    // pose_1 << 0.0, 0.0, 0.0,3,
    //           0.0, 0.0, 0.0,3,
    //           0.0, 0.0, 0.0,0,
    //           0  , 0  , 0  ,1;

    // Sphere *sphere_1 = new Sphere(pose_1, radius_1);
            
    // monitor1.addObstacle(sphere_1);

    Eigen::Vector3d initPose;
    initPose[0]=0.0; // x position of center of box
    initPose[1]=0.0; // y position of center of box
    initPose[2]=1.5; // z position of center of box
    base1.updatePose(initPose);

    // Create the baseController class based off the first monitor

    BaseController baseController1(&monitor1, K, D, gamma, beta);
    
    // Init ROS listener
    ros::Subscriber goalSub = n.subscribe(goalTopic, 1000, &BaseController::goalCallback, &baseController1);
    ros::Subscriber obstacleSub = n.subscribe("kinova_controller/obstacles", 1000, &BaseController::updateObstacles, &baseController1);
    ros::Publisher obstaclePub_sim = n.advertise<visualization_msgs::Marker>("kinova_controller/obstacles", 1000);
    ros::Subscriber basePosSub = n.subscribe("/odom", 1000, &BaseController::baseCallback, &baseController1);

    ros::Publisher markersPub = n.advertise<visualization_msgs::Marker>("kinova_controller/markers", 1000);
    ros::Publisher baseVelPub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    ros::Rate loop_rate(10); 


    
    geometry_msgs::Twist baseVelocity;
    
    while(ros::ok()) {

    //    MarkerPublisher mPublisher(obstaclePub_sim, visualization_msgs::Marker::SPHERE, "base_link", "obstacles", 0, 3, 3, 0, 255, 2, 0, 0.5);
    //    mPublisher.setRadius(0.3);

    //    mPublisher.Publish();

       baseVelocity = baseController1.control_loop();
       
       
        baseVelPub.publish(baseVelocity);
        for(int i=0; i<baseController1.rvizObstacles.size(); i++){
            markersPub.publish(baseController1.rvizObstacles[i]->marker);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}






//Eigen::Vector3d  Box3::OwnClosestPoint(Sphere *sphere)
// {    
//      Eigen::Vector3d l_pt1[8];
//      Eigen::Vector3d l_pt2[6];
//      Eigen::Vector3d sideCenters[6];
//      Eigen::Vector3d Corners[8];
//     double  distance;
//     double min_distance;
//     Eigen::Vector3d closest_point;


//       for (int i = 0; i < 8; i++)
//     {
        
//            Corners[i]=this->CornerPoint(i); 
             
//         // cout<< "Corner point " <<i<<" = "<<endl<<Corners[i]<<endl;
          
//     }


   
//       for (int i = 0; i < 6; i++)
//     {
        
//            sideCenters[i]=this->SideCenterPoint(i); 
             
//         // cout<< "Corner point " <<i<<" = "<<endl<<Corners[i]<<endl;
          
//     }
     

// for (int i = 0; i < 8; i++)
// {     
//       Eigen::Vector3d c_pt = Corners[i];
//       l_pt1[i] = sphere->getClosestPointToPoint(c_pt);
           
      
//       distance = std::sqrt((std::pow(l_pt1[i][0]-c_pt[0],2)+std::pow(l_pt1[i][1]-c_pt[1],2)+std::pow(l_pt1[i][2]-c_pt[2],2))*1.0);
//       if (distance<min_distance){
//           min_distance=distance;
//           closest_point= c_pt;
//       }


// }



// for (size_t i = 0; i < 6; i++)
// {
    
//       Eigen::Vector3d c_pt = sideCenters[i];
//       l_pt2[i] = sphere->getClosestPointToPoint(c_pt);
           
      
//       distance= std::sqrt((std::pow(l_pt2[i][0]-c_pt[0],2)+std::pow(l_pt2[i][1]-c_pt[1],2)+std::pow(l_pt2[i][2]-c_pt[2],2))*1.0);
//       if (distance<min_distance){
//           min_distance=distance;
//           closest_point= c_pt;
//       }
// }



// // std::cout<<"minmum distance to line from box : "<<min_distance<< std::endl;
 
//  return closest_point;

// }
