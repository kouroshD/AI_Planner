#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <string>
//#include <iterator>
#include <fstream>

#include <sys/types.h>
#include <sys/stat.h>
#include <andor_msgs/andorSRV.h>
#include <andor_msgs/Hyperarc.h>
#include <andor_msgs/Node.h>
#include "knowledge_msgs/knowledgeSRV.h"
#include "robot_interface_msgs/Joints.h"
#include "robot_interface_msgs/SimulationRequestMsg.h"
#include "robot_interface_msgs/SimulationResponseMsg.h"
#include "basic_classes.hpp"


#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST

using namespace std;

//*********************
class seq_planner_class{
public:

	seq_planner_class(string seqPlannerPath,string AssemblyName);
	~seq_planner_class();
	void GenerateStateActionTable(vector<vector<string>> gen_Feasible_state_list, vector<int> gen_Feasible_stateCost_list, string graphName, bool graphSolved);
	bool updateAndor; // it is true if we want to update the andor in here, or in andor graph module.
	bool nodeSolved;// it is true if we have a solved node, so we should update the andor graph.
	bool haSolved;// it is true if we have a solved hyperarc, so we should update the andor graph.
    vector<string> Solved_node_list; // the list of all the solved node at the current moment
    vector<string> Solved_hyperarc_list;// the list of all the solved hyperarc at the current moment
    string seq_planner_path; // the path to the files of the seq_planner
    string assembly_name;//
    string AndOrUpdateName;




private:
	//! the length of the vector is equal to number of agents, if the agent[i] is responsible is true, otherwise it is false;
	vector<agent> agents;
	vector<actionDef> action_Definition_List;    // list of definition of the actions
    vector<offline_state_action_graph>  Full_State_action_list;// list of all the actions for all the states, this list should be found offline by a planner
    vector<feasible_state_action> state_action_table;
    vector<string> objectTypeVector; // list of object type in the world to be instantiated

    ofstream fileLog;
    double timeNow;
//    	const char* DataLogPath	="/home/nasa/Datalog/Mechatronics/user1/1";
//    	string DataLogPath2		="/home/nasa/Datalog/Mechatronics/user1/1";
//    	mkdir(DataLogPath, S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
//    	Myfile1.open ((DataLogPath2+"/Assembly_Timing.txt").c_str(),ios::app);


    vector<optimal_state_simulation> simulation_vector;
    vector<string> complexActionsList;
    bool emergencyFlag;
	int optimal_state;
	int next_action_index;
//	int agent_update;// the agent number who arrives the latest ack.
	int simulationVectorNumber,SimulationActionNumber;
	vector<string> hierarchicalGraphList;// the list of the hierarchical and/or graphs

	ros::NodeHandle nh;
	ros::Subscriber subHumanActionAck;
	ros::Subscriber subRobotActionAck;
	ros::Subscriber subSimulationAck;
	ros::Publisher pubRobotCommand;
	ros::Publisher pubSimulationCommand;
	ros::ServiceClient knowledgeBase_client;

	void CallBackHumanAck(const std_msgs::String::ConstPtr& msg);
	void CallBackRobotAck(const std_msgs::String::ConstPtr& msg);
	void PublishHumanAction(string ActionName, string AgentsName, vector<string> ColleaguesName);
	void PublishRobotAction(string ActionName, vector<string> AgentName, vector<string> ColleaguesName);
	bool CanAgentPerformAction(vector<string> agent_name,string agent_type, string action_name, bool sufficiency);

	void SetActionDefinitionList(string actionDefinitionPath);
	void SetAgentsList(void);
	void SetStateActionList(string stateActionPath, string andorGraphName, vector<offline_state_action> & offline_state_action_list);
	void CheckStateActionList(void);
	void ReadObjectsType(string objTypePath);

	void UpdateStateActionTable(string ActionName, vector<string>AgentsName, bool success);
	void UpdateStateActionTable_ComplexAction(string ActionName, bool success);
//	agent_update --> the agent number who arrives the latest ack.
	void FindOptimalState(void);
	void FindNextAction(void);
	void FindResponisibleAgent(void);
	string ResponsibleAgentType(string agent_name); //! it returns if an agent is Human/Robot
	void CheckStateExecution(void);
	void EmergencyRobotStop(void);
	void UpdateRobotEmergencyFlag(string ActionName, vector<string>AgentsName, bool success);

	void GenerateOptimalStateSimulation(void); // here we have all the simulation vector with assigned agents or parameters.
	void UpdateSimulation(const robot_interface_msgs::SimulationResponseMsg&  simulationResponse); // here we update the simulation results and give one by one the next command to robot simulator
	void GiveSimulationCommand(void);
	void RankSimulation(void); // when the simulation is done, we rank them, and we assign the agents and action parameters to the optimal state representation, and we call the FindNextAction function.
	// if agent could not perform the actions that was simulated, we make that state infeasible.

};

//****************************************
void Print2dVec(vector<vector<string>> vec ){
	cout<<"*** print Vector ::string ***"<<endl;
	for(int i=0;i<vec.size();i++)
	{
		for(int j=0;j<vec[i].size();j++)
			cout<<vec[i][j]<<" ";
		cout<<endl;
	}
};
void PossibileCombinations(vector<string> input, int combination_number, vector<string> & output){
	cout<<"*** PossibileCombinations ***"<<endl;
	// no repetition, orders are important
	string connectionStr="+";
	output.clear();
	if(combination_number==1)
	{
		output=input;
	}
	else if(combination_number==2)
	{
		for(int i=0;i<input.size();i++){
			for(int j=i+1;j<input.size();j++){
				output.push_back(input[i]+"+"+input[j]);
				output.push_back(input[j]+"+"+input[i]);
			}
		}
	}
	else
	{
		cout<<"Not yet implemented"<<endl;
	}
};

bool isTwoActionsEqual(const action& action1, const action& action2){
	bool temp_equality=true;
	// terms of equality: action names, actions Agents, actions parameters

	if(action1.name!=action2.name)
		temp_equality=false;
	if(action1.assigned_agents.size()!=action2.assigned_agents.size())
		temp_equality=false;
	if(action1.assignedParameters.size()!=action2.assignedParameters.size())
		temp_equality=false;

	if(temp_equality==true)
	{
		for(int i=0;i<action1.assigned_agents.size();i++)
		{
			if(action1.assigned_agents[i]!=action2.assigned_agents[i])
				temp_equality=false;
		}
	}
	if(temp_equality==true)
	{
		for(int i=0;i<action1.assignedParameters.size();i++)
		{
			if(action1.assignedParameters[i]!=action2.assignedParameters[i])
				temp_equality=false;
		}
	}

	return temp_equality;
};



