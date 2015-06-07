/**
 * @file    localization_node.cpp
 * @author  George Andrew Brindeiro and Mateus Mendelson
 * @date    04/10/2012
 *
 * @attention Copyright (C) 2012
 * @attention Laboratório de Automação e Robótica (LARA)
 * @attention Universidade de Brasília (UnB)
 */

// artoolkit_localization headers
#include <artoolkit_localization/localization_node.h>
#include <artoolkit_localization/ukf.h>

// Standard C libraries
#include <csignal>
#include <cstdio>

// ROS libraries
#include <ros/ros.h>

#include <ar_pose/ARMarkers.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <ros/package.h>
#include <visualization_msgs/Marker.h>

 //Please, delete me!
 double g_theta;

std::vector<double> controls;
std::map< int, vector<double> > measurements;

UKF* localizer;

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#define SHAPE visualization_msgs::Marker::CUBE;

void setupRealMap(ros::Publisher& marker_pub)
{
    //Altura da câmera em relação ao chão = 92 cm
    //Altura do chão até o teto = 273 cm
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now ();
    marker.ns = "markers";
    
    /* ROOF */
    marker.id = 0;
    
    marker.type = SHAPE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = 2.4;
    marker.pose.position.y = 2.4;
    marker.pose.position.z = 2.95;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 6;
    marker.scale.y = 6;
    marker.scale.z = 0.01;
    
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 1 */
    marker.id = 1;
    
    marker.type = SHAPE;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 2.94;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    
    marker.scale.x = 0.125;
    marker.scale.y = 0.125;
    marker.scale.z = 0.01;
    
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 2 */
    marker.id = 2;
    
    marker.pose.position.x = 0.7;
    marker.pose.position.y = 0.0;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 3 */
    marker.id = 3;
    
    marker.pose.position.x = 1.6;
    marker.pose.position.y = 0.0;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 4 */
    marker.id = 4;
    
    marker.pose.position.x = 2.1;
    marker.pose.position.y = 0.0;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 5 */
    marker.id = 5;

    marker.pose.position.x = 2.7;
    marker.pose.position.y = 0.0;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 6 */
    marker.id = 6;
    
    marker.type = SHAPE;
    
    marker.pose.position.x = 3.3;
    marker.pose.position.y = 0.0;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 7 */
    marker.id = 7;
    
    marker.pose.position.x = 3.9;
    marker.pose.position.y = 0.0;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 8 */
    marker.id = 8;
    
    marker.pose.position.x = 4.5;
    marker.pose.position.y = 0.0;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 9 */
    marker.id = 9;
    
    marker.pose.position.x = 0.3;
    marker.pose.position.y = 1.8;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 10 */
    marker.id = 10;
    
    marker.pose.position.x = 0.3;
    marker.pose.position.y = 2.6;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 11 */
    marker.id = 11;
    
    marker.pose.position.x = 0.3;
    marker.pose.position.y = 3.1;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 12 */
    marker.id = 12;
    
    marker.pose.position.x = 0.3;
    marker.pose.position.y = 3.7;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 13 */
    marker.id = 13;
    
    marker.pose.position.x = 0.3;
    marker.pose.position.y = 4.2;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 14 */
    marker.id = 14;
    
    marker.pose.position.x = 0.4;
    marker.pose.position.y = 4.8;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 15 */
    marker.id = 15;
    
    marker.pose.position.x = 1.1;
    marker.pose.position.y = 4.8;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 16 */
    marker.id = 16;
    
    marker.pose.position.x = 1.7;
    marker.pose.position.y = 4.8;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 17 */
    marker.id = 17;
    
    marker.pose.position.x = 2.3;
    marker.pose.position.y = 4.8;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 18 */
    marker.id = 18;
    
    marker.pose.position.x = 2.9;
    marker.pose.position.y = 4.8;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 19 */
    marker.id = 19;
    
    marker.pose.position.x = 3.5;
    marker.pose.position.y = 4.8;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 20 */
    marker.id = 20;
    
    marker.pose.position.x = 4.1;
    marker.pose.position.y = 4.8;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 21 */
    marker.id = 21;
    
    marker.pose.position.x = 4.2;
    marker.pose.position.y = 4.2;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 22 */
    marker.id = 22;
    
    marker.pose.position.x = 4.2;
    marker.pose.position.y = 3.7;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 23 */
    marker.id = 23;
    
    marker.pose.position.x = 4.2;
    marker.pose.position.y = 3.1;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 24 */
    marker.id = 24;
    
    marker.pose.position.x = 4.2;
    marker.pose.position.y = 2.6;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 25 */
    marker.id = 25;
    
    marker.pose.position.x = 4.2;
    marker.pose.position.y = 1.9;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 26 */
    marker.id = 26;
    
    marker.pose.position.x = 4.3;
    marker.pose.position.y = 1.0;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 27 */
    marker.id = 27;
    
    marker.pose.position.x = 4.8;
    marker.pose.position.y = 1.9;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 28 */
    marker.id = 28;
    
    marker.pose.position.x = -0.3;
    marker.pose.position.y = 0.7;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 29 */
    marker.id = 29;
    
    marker.pose.position.x = -0,3;
    marker.pose.position.y = 1.2;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    /* 30 */
    marker.id = 30;
    
    marker.pose.position.x = 0.9;
    marker.pose.position.y = 1.8;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
}

//void publish_map_markers(ros::Publisher& marker_pub)
void publishMapMarkers(UKF* localizer, ros::Publisher& marker_pub)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now ();
    marker.ns = "markers";
    marker.id = 31;
    
    marker.type = SHAPE
    marker.action = visualization_msgs::Marker::ADD;
    
    #if POSE_WITH_COVARIANCE_STAMPED
        marker.pose.position.x = localizer->poseMsg().pose.pose.position.x;
        marker.pose.position.y = localizer->poseMsg().pose.pose.position.y;
        marker.pose.position.z = localizer->poseMsg().pose.pose.position.z;
    #elif POSE_STAMPED
        marker.pose.position.x = localizer->poseMsg().pose.position.x;
        marker.pose.position.y = localizer->poseMsg().pose.position.y;
        marker.pose.position.z = localizer->poseMsg().pose.position.z;
    #endif
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.125;
    marker.scale.y = 0.125;
    marker.scale.z = 0.01;
    
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
   /* visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "map";
    marker.id = 4;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 1.8;
    marker.pose.position.y = 0;
    marker.pose.position.z = 2.75;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.13;
    marker.scale.y = 0.13;
    marker.scale.z = 0.03;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    marker_pub.publish(marker);*/
}

void pubPath(UKF* localizer, ros::Publisher& marker_pub, int i)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now ();
    marker.ns = "estimated_path";
    
    marker.id = i;
    
    marker.type = 0;
    marker.action = visualization_msgs::Marker::ADD;
    
    #if POSE_WITH_COVARIANCE_STAMPED
        marker.pose.position.x = localizer->poseMsg().pose.pose.position.x;
        marker.pose.position.y = localizer->poseMsg().pose.pose.position.y;
        marker.pose.position.z = localizer->poseMsg().pose.pose.position.z;
        marker.pose.orientation.x = localizer->poseMsg().pose.pose.orientation.x;
        marker.pose.orientation.y = localizer->poseMsg().pose.pose.orientation.y;
        marker.pose.orientation.z = localizer->poseMsg().pose.pose.orientation.z;
        marker.pose.orientation.w = localizer->poseMsg().pose.pose.orientation.w;
    #elif POSE_STAMPED
        marker.pose.position.x = localizer->poseMsg().pose.position.x;
        marker.pose.position.y = localizer->poseMsg().pose.position.y;
        marker.pose.position.z = localizer->poseMsg().pose.position.z;
        marker.pose.orientation.x = localizer->poseMsg().pose.orientation.x;
        marker.pose.orientation.y = localizer->poseMsg().pose.orientation.y;
        marker.pose.orientation.z = localizer->poseMsg().pose.orientation.z;
        marker.pose.orientation.w = localizer->poseMsg().pose.orientation.w;
    #endif
    

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
}

int main(int argc, char **argv)
{
    int i = 31;
    
    // Setup signal handlers
    setupSigHandler();

    /* Node initialization */

    // Setup ROS structures
    ros::init(argc, argv, "localization_node");
    ros::NodeHandle loc_nh;
    
    // Subscribed topics
    ros::Subscriber sub_odom = loc_nh.subscribe<nav_msgs::Odometry>("pose", 1, twistCallback);
    ros::Subscriber sub_markers = loc_nh.subscribe<ar_pose::ARMarkers>("ar_pose_marker", 1, arPoseMarkerCallback);
    
    // Published topics
    #if POSE_WITH_COVARIANCE_STAMPED
        ros::Publisher pub_pose = loc_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("artoolkit_localization/pose", 1);
    #elif POSE_STAMPED
        ros::Publisher pub_pose_no_cov = loc_nh.advertise<geometry_msgs::PoseStamped>("artoolkit_localization/pose_no_cov", 1);
    #endif

    // Broadcaster for camera/robot transform
    tf::Transform transform;
    tf::TransformBroadcaster br;

    // Node loop rate
    ros::Rate loop_rate(100);

    /* UKF */
    //ROS_DEBUG("Path to ARMap is not absolute... Possible SEGFAULT!");
    string path = ros::package::getPath ("artoolkit_localization");

    #ifdef MAPPING
        path.append ("/cfg/outputMap.map");
    #else
        path.append ("/cfg/lara.map");
    #endif

    localizer = new UKF(0.1, path);

    /* MAP MARKERS */
    ros::Publisher marker_pub = loc_nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    while(ros::ok())
    {
        ros::Time ts = ros::Time::now();

        /* Node main loop */
        if(controls.size() > 0 || measurements.size() > 0)
        {
            localizer->localize(controls,measurements);
            controls.clear();
            measurements.clear();
        }
        else
            ROS_DEBUG("No controls or measurements available.");

        // Publish current pose estimate
        #if POSE_WITH_COVARIANCE_STAMPED
            pub_pose.publish(localizer->poseMsg());
        #elif POSE_STAMPED
            pub_pose_no_cov.publish(localizer->poseMsg());
        #endif

        ////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Printing current estimated pose

        #if POSE_WITH_COVARIANCE_STAMPED
            /*ROS_INFO("x = %lf", localizer->poseMsg().pose.pose.position.x);
            ROS_INFO("y = %lf", localizer->poseMsg().pose.pose.position.y);
            ROS_INFO("z = %lf", localizer->poseMsg().pose.pose.position.z);
            ROS_INFO("g_theta = %lf\n\n", g_theta);*/
            
            
            cout << "\nx = " << localizer->poseMsg().pose.pose.position.x << endl;
            cout << "y = " << localizer->poseMsg().pose.pose.position.y << endl;
            cout << "z = " << localizer->poseMsg().pose.pose.position.z << endl;
            cout << "g_theta = " << g_theta << endl;
            
        #elif POSE_STAMPED
            /*
            cout << "\nx = " << localizer->poseMsg().pose.position.x << endl;
            cout << "y = " << localizer->poseMsg().pose.position.y << endl;
            cout << "z = " << localizer->poseMsg().pose.position.z << endl;
            cout << "theta = " << g_theta << endl;
            //cout << "theta = " << tf::getYaw(localizer->poseMsg().pose.orientation) << endl;
            */
        #endif
        
        
        setupRealMap(marker_pub);
        //Publishing path
        //i++;
        //pub_path(localizer, marker_pub, i);
        publishMapMarkers(localizer, marker_pub);
        ////////////////////////////////////////////////////////////////////////////////////////////////////////

        // odom to pose_no_cov
        tf::Quaternion rotation;
        #if POSE_WITH_COVARIANCE_STAMPED
            tf::quaternionMsgToTF(localizer->poseMsg().pose.pose.orientation, rotation);
            transform.setOrigin( tf::Vector3(localizer->poseMsg().pose.pose.position.x, localizer->poseMsg().pose.pose.position.y, 0) );
        #elif POSE_STAMPED
            tf::quaternionMsgToTF(localizer->poseMsg().pose.orientation, rotation);
            transform.setOrigin( tf::Vector3(localizer->poseMsg().pose.position.x, localizer->poseMsg().pose.position.y, 0) );
        #endif

        transform.setRotation(rotation);
        br.sendTransform(tf::StampedTransform(transform, ts, "odom", "pose_no_cov"));

        // Publish world/camera tf transform based on robot position
        // Height: 0.85 Rotation: -90º in z
        // Robot x axis is front, y axis is left
        // Camera x axis is right, y axis is front (both in robot's perspective)
        transform.setOrigin( tf::Vector3(0, 0, 0.85) );
        transform.setRotation( tf::createQuaternionFromRPY(0, 0, -1.57) );
        br.sendTransform(tf::StampedTransform(transform, ts, "base_link", "camera"));

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void twistCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    controls.clear();

    ROS_DEBUG("Received twist message!");

    double v = msg->twist.twist.linear.x;
    double w = msg->twist.twist.angular.z;

    controls.push_back(v);
    controls.push_back(w);

    ROS_DEBUG("(v,w) = (%lf,%lf)", v, w);
}

void arPoseMarkerCallback(const ar_pose::ARMarkers::ConstPtr& msg)
{
    measurements.clear();

    std::vector<ar_pose::ARMarker> markers = msg->markers;
    
    int num_markers = markers.size();
    
    if(num_markers > 0)
    {
        if(num_markers == 1)
            ROS_DEBUG("Received %d ar_pose_marker message!", num_markers);
        else
            ROS_DEBUG("Received %d ar_pose_marker messages!", num_markers);
        
        for(std::vector<ar_pose::ARMarker>::iterator marker_it = markers.begin(); marker_it != markers.end(); ++marker_it)
        {
            int num_marker = marker_it-markers.begin();
            uint32_t id = marker_it->id;
            double dx = marker_it->pose.pose.position.y; // OBS: inversion due to difference in camera/body frame orientation
            double dy = -marker_it->pose.pose.position.x;
            double theta = localizer->state_(2,0);

            // Just making tests. Make sure to delete the next 3 lines
            g_theta = tf::getYaw(marker_it->pose.pose.orientation);
            //ROS_INFO("theta = %f\n", g_theta);
            //cout << "Olha o theta! " << theta << endl;

            measurements[id].push_back(UKF::range(dx,dy));
            measurements[id].push_back(UKF::bearing(dx,dy,theta));

            ROS_DEBUG("Marker #%d", num_marker);
            ROS_DEBUG("id: %d", id);
            ROS_DEBUG("Position: (%lf,%lf)", dx, dy);
            ROS_DEBUG("Range/Bearing: (%lf,%lf)", measurements[id][0], measurements[id][1]);
        }
    }
}

void setupSigHandler()
{
    signal(SIGSEGV, &sigHandler);
    signal(SIGINT, &sigHandler);
    signal(SIGTSTP, &sigHandler);
}

void sigHandler(int sig)
{
    switch(sig)
    {
        case SIGSEGV:
            signal(SIGSEGV, SIG_DFL);
            printf("Signal caught: SIGSEGV\n");
            break;
        case SIGINT:
            signal(SIGINT, SIG_IGN);
            printf("Signal caught: SIGINT\n");
            break;
        case SIGTSTP:
            signal(SIGTSTP, SIG_IGN);
            printf("Signal caught: SIGTSTP\n");
            break;
    }

    printf("Closing artoolkit_localization nicely...\n");
    
    exit(0);
}
