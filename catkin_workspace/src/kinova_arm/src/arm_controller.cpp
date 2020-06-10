#include "arm_controller.h"

CollisionMonitor::CollisionMonitor(Monitor monitorObject){
  Eigen::Matrix4d endpose;
  this->monitor = monitorObject;
  Arm arm = this->monitor.arm;
  
}