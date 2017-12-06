//===============================================================================//
// Name
// Author(s)	 :  Kourosh.darvish@edu.unige.it
// Affiliation   : University of Genova, Italy - dept. DIBRIS
// Version		 :
// Description   :
//===============================================================================//

/*!
 *  This code is for managing the tasks and actions and planning the actions for HRI
 * 	It considers the actions of all the agents are sequential, so each moment, one agent is performing a action.
 *  It is also possible, an agent can not perform an assigned action to it.
 *  It gets info of the objects in the working space from knowledge base, which now is defined as a service
 *
 * */

#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <andor_msgs/andorSRV.h>
#include <andor_msgs/Hyperarc.h>
#include <andor_msgs/Node.h>
#include <boost/shared_ptr.hpp>

#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST

using namespace std;

int main(int argc, char **argv)
{
	// ROS parameters definitions:
	ros::init(argc, argv, "seq_planner");
	ros::NodeHandle nh;
	ros::spin();
	int hri_ros_freq=80;//hz
	ros::Rate loop_rate(hri_ros_freq);

	// AND/OR graph service definition:

	ros::ServiceClient andorSRV_client = nh.serviceClient<andor_msgs::andorSRV>("andorService");
	andor_msgs::andorSRV andor_srv;

	/*! 	AGENTS		*/
	int responsibleAgent=0; //! it defines which agent is responsible at each moment, 0: human, 1: robot (in this example we have two agents)
	int ack_agent=0;		//! it shows which agent returns an acknowledgment at this time:( 0: human, 1: robot )


	int count=0;
	while (ros::ok())
	{





		if (count==0){	usleep(0.5e6); }
		loop_rate.sleep();
		count++;
		ros::spinOnce();
	}

	return 1;

}   

