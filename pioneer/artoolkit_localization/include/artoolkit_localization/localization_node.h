/**
* @file		localization_node.h
* @author	George Andrew Brindeiro and Mateus Mendelson
* @date		04/10/2012
*
* @brief ROS node for UKF localization using ARToolkit markers
*
* This node uses the information provided by the ar_pose and p2os packages to 
* obtain robot localization using the Unscented Kalman Filter, with ARToolkit 
* markers placed on the ceiling acting as landmarks in a pre-defined map.
* 
* Contact: georgebrindeiro@lara.unb.br
* 
* Revisions:
* [04/10/2012] Created
*/

#ifndef LOCALIZATION_NODE_H
#define LOCALIZATION_NODE_H

#include <ar_pose/ARMarkers.h>
#include <nav_msgs/Odometry.h>

/* Function prototypes etc. here */

void twistCallback(const nav_msgs::Odometry::ConstPtr& msg);
void arPoseMarkerCallback(const ar_pose::ARMarkers::ConstPtr& msg);

void setupSigHandler();
void sigHandler(int sig);

#endif //LOCALIZATION_NODE_H
