#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <boost/algorithm/string.hpp>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>
#include <string>
//#include <iterator>
//#include <sstream>
#include <fstream>
#include <andor_msgs/andorSRV.h>
#include <andor_msgs/Hyperarc.h>
#include <andor_msgs/Node.h>

#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST

using namespace std;

class action{
public:
	string name;
	vector<string> agents; // the agents who "can" perform the action
	string actionType; // if it is simple, or complex (an andor graph)
	string actionMode; // if the action should be performed by single agent, all the agents, or jointly between agents
	action(void)
	{
		name="";
		actionType="";
		actionMode="";
	};
	~action(){};
	void Print(void){
		cout<<"******** actions info *********"<<endl;
		cout<<"name: "<<name<<endl;
		cout<<"actionType: "<<actionType<<endl;
		cout<<"actionMode: "<<actionMode<<endl;
		for (int i=0;i<agents.size();i++)
			cout<<agents[i]<<" ";
		cout<<endl;

	};
};
//****************************
class agent{
public:
	string name;
	string lastAssignedAction;
	string lastActionAck;
	bool allowToChangePath;
	bool isBusy;
	bool isSuccessfullyDone;

	agent(void)
	{
		name="";
		lastAssignedAction="";
		lastActionAck="";
		allowToChangePath=false;
		isBusy=false;
		isSuccessfullyDone=false;
	};
	~agent(){};
	void Print(void){
		cout<<"******** agent info *********"<<endl;
		cout<<name<<name<<endl;
		cout<<"lastAssignedAction: "<<lastAssignedAction<<endl;
		cout<<"lastActionAck: "<<lastActionAck<<endl;
		cout<<"allowToChangePath: "<<allowToChangePath<<endl;
		cout<<"isBusy: "<<isBusy<<endl;
	};
};
//****************************


class seq_planner_class{
public:

	seq_planner_class(string actionDefinitionPath, string stateactionPath);
	~seq_planner_class();
	void GenerateStateActionTable(vector<vector<string>> gen_Feasible_state_list, vector<int> gen_Feasible_stateCost_list);
	bool updateAndor; // it is true if we want to update the andor in here, or in andor graph module.
	bool nodeSolved;// it is true if we have a solved node, so we should update the andor graph.
	bool haSolved;// it is true if we have a solved hyperarc, so we should update the andor graph.
    vector<string> Solved_node_list; // the list of all the solved node at the current moment
    vector<string> Solved_hyperarc_list;// the list of all the solved hyperarc at the current moment


private:

	//! the length of the vector is equal to number of agents, if the agent[i] is responsible is true, otherwise it is false;
	vector<agent> agents;
	vector<action> action_Definition_List;    // list of definition of the actions
    vector<vector<string>> Full_State_action_list; // list of all the actions for all the states

    vector<vector<string>> Feasible_states_actions_table;     // table of the feasible state-actions names
    vector<vector<bool>> Feasible_states_actions_progress;     // table of the feasible state-actions done(true)/not done (false)actions
    vector<vector<string>> Feasible_states_Names; // the name of the feasible states , type of it :nodes / hyperacs
    vector<int> Feasible_States_cost; // weight for each feasible state
    vector<int> Feasible_states_isFeasible;// if still the Feasible given by the and/or graph is feasible (true) or not (false) ?
	int optimal_state;
	int next_action_index;
	int agent_update;// the agent number who arrives the latest ack.

	ros::NodeHandle nh;
	ros::Subscriber subHumanActionAck;
	ros::Subscriber subRobotActionAck;
	ros::Publisher pubRobotCommand = nh.advertise<std_msgs::String>("hri_robot_command",80);


	void CallBackHumanAck(const std_msgs::String::ConstPtr& msg);
	void CallBackRobotAck(const std_msgs::String::ConstPtr& msg);
	void PublishHumanAction(void);
	void PublishRobotActionLeftArm(void);
	void PublishRobotActionRightArm(void);
	void PublishRobotActionJointly(void);

	void SetActionDefinitionList(string actionDefinitionPath);
	void SetStateActionList(string stateActionPath);
	void SetAgentsList(void);

	void UpdateStateActionTable(void);
	void FindNextAction(void);
	void FindResponisibleAgent(void);
	void CheckStateExecution(void);
};

//****************************************
void Print2dVec(vector<vector<string>> vec ){
	cout<<"*** print Vector ***"<<endl;
	for(int i=0;i<vec.size();i++)
	{
		for(int j=0;j<vec[i].size();j++)
			cout<<vec[i][j]<<" ";
		cout<<endl;
	}
};
