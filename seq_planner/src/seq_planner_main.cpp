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
#include "seq_planner_class.hpp"

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

	ros::Rate loop_rate(80);

	// planner class initialization:
	string AssemblyName="TableAssembly";
	const char* home=getenv("HOME");
	string assemblyPath(home);
	assemblyPath=assemblyPath+"/catkin_ws/src/AI_PLANNER/seq_planner/files/TableAssembly";

	seq_planner_class plan_obj(assemblyPath, AssemblyName);
	vector<vector<string>> gen_Feasible_state_list;
	vector<int> gen_Feasible_stateCost_list;

	// ANDOR graph service definition:

	ros::ServiceClient andorSRV_client = nh.serviceClient<andor_msgs::andorSRV>("andorService");


	/*! 	AGENTS		*/
	int responsibleAgent=0; //! it defines which agent is responsible at each moment, 0: human, 1: robot (in this example we have two agents)
	int ack_agent=0;		//! it shows which agent returns an acknowledgment at this time:( 0: human, 1: robot )

	/*! ANDOR graph update flags and parameters*/
	bool isGraphSolved=false; // if it is true the hri task is done
	int count=0;
	while (ros::ok())
	{

//		cout<<100<<endl;
//		cout<<100<<" "<<plan_obj.updateAndor<<" "<<plan_obj.nodeSolved <<" "<<plan_obj.haSolved <<endl;
		if (plan_obj.updateAndor==true && count>0)
		{
			andor_msgs::andorSRV andor_srv;
			cout<<"101: "<<plan_obj.AndOrUpdateName<<endl;
			andor_srv.request.graphName=plan_obj.AndOrUpdateName;
			if(plan_obj.nodeSolved==true)
			{
				cout<<102<<endl;
				for(int i=0; i<plan_obj.Solved_node_list.size(); i++)
				{
					andor_srv.request.solvedNodes.push_back(plan_obj.Solved_node_list[i]);
				}
				plan_obj.Solved_node_list.clear();
				plan_obj.nodeSolved=false;
			}

			if(plan_obj.haSolved==true)
			{
				for(int i=0; i<plan_obj.Solved_hyperarc_list.size(); i++)
				{
					andor_srv.request.solvedHyperarc.push_back(plan_obj.Solved_hyperarc_list[i]);
				}
				plan_obj.Solved_hyperarc_list.clear();
				plan_obj.haSolved=false;
			}

			if (andorSRV_client.call(andor_srv))
			{
				cout<<103<<endl;
				isGraphSolved=andor_srv.response.graphSolved;
				if(isGraphSolved==true && plan_obj.AndOrUpdateName==AssemblyName)
				{
					cout<<	FGRN("The Assembly task is Done")<<endl;
					return 1;
				}
				else
				{
					cout<<104<<endl;

					for (int i=0;i<andor_srv.response.feasibleNodes.size();i++)
					{
						cout<<105<<endl;
						vector<string>Feasible_state;
						string feasbible_state_name=andor_srv.response.feasibleNodes[i].nodeName;
						Feasible_state.push_back(feasbible_state_name);
						Feasible_state.push_back("Node");
						int cost=andor_srv.response.feasibleNodes[i].nodeCost;
						gen_Feasible_stateCost_list.push_back(cost);
						gen_Feasible_state_list.push_back(Feasible_state);
					}
					for (int i=0;i<andor_srv.response.feasibleHyperarcs.size();i++)
					{
						cout<<106<<endl;
						vector<string>Feasible_state;
						string feasbible_state_name=andor_srv.response.feasibleHyperarcs[i].hyperarcName;
						Feasible_state.push_back(feasbible_state_name);
						Feasible_state.push_back("Hyperarc");
						int cost=andor_srv.response.feasibleHyperarcs[i].hyperarcCost;
						gen_Feasible_stateCost_list.push_back(cost);
						gen_Feasible_state_list.push_back(Feasible_state);
					}
					plan_obj.updateAndor=false;
					plan_obj.GenerateStateActionTable(gen_Feasible_state_list,gen_Feasible_stateCost_list,plan_obj.AndOrUpdateName, isGraphSolved );
					gen_Feasible_state_list.clear();
					gen_Feasible_stateCost_list.clear();
				}

			}

		}


		// if you do not put at the beginning, it does not start publishing immidiately
		if (count==0){	usleep(0.5e6); }
//		cout<<count<<endl;
		loop_rate.sleep();
		count++;
		ros::spinOnce();
	}

	return 1;

}   

