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

//class offline_state_action;
//
//class actionDef{
//public:
//	string name;
//	vector<vector<string>> possible_agents; // the agents who "can" perform the action// each raw if the size is one single action, otherwise it is joint action
//	string actionType; // if it is simple, or complex (a complex action is another andor graph)
//	string actionMode; // if the action should be performed by single agent or jointly between agents
//	vector <string> parameterTypes; // what is the feature of an argument we should look for in the knowledge base,
//	// for example: if we say: approach plate, in fact we should look for the grasping pose of the plate.
//	// it should be given in order
//	vector<offline_state_action> ComplexAction_state_action_list;//
//	actionDef(void)
//	{
//		name="";
//		actionType="";
//		actionMode="single";
//	};
//	~actionDef(){};
//	void Print(void){
//		cout<<">>>>>>>> actionDef info"<<endl;
//		cout<<"name: "<<name<<endl;
//		cout<<"actionType: "<<actionType<<endl;
//		cout<<"actionMode: "<<actionMode<<endl;
//		cout<<"possible Agents: ";
//		for (int i=0;i<possible_agents.size();i++)
//		{
//			for (int j=0; j<possible_agents[i].size();j++)
//				cout<<possible_agents[i][j]<<" ";
//			cout<<" | ";
//		}
//		cout<<endl;
//
//		cout<<"Action Argument Feature: ";
//		for (int i=0;i<parameterTypes.size();i++)
//		{
//				cout<<parameterTypes[i]<<" ";
//		}
//		cout<<endl;
//
//		cout<<" complex action state-action list:"<<endl;
//		for (int i=0;i<ComplexAction_state_action_list.size();i++)
//		{
//			ComplexAction_state_action_list[i].Print();
//		}
//	};
//};
////
//class action{
//public:
//	actionDef &refActionDef;
//	string name;
//	vector<string> assigned_agents;
//	vector<string> assignedParameters; // Example: object1-graspingPose1, Point1
//	vector<string> GeneralParameters;
//	vector<bool> isDone; // it is a vector, because for some joint actions (human+robot) it needs to fill all of them in order to say an action is done;
//	string actionAndParameters; //Example:  Approach_object1-graspingPose1, Approach_Point1
//	string Action_GeneralParameters;//Example:  Appraoch_object1, Approach_Point1
//
//
//	action(actionDef &actionDefObj):refActionDef(actionDefObj){
//		name=refActionDef.name;
//		actionAndParameters="--";
//	};
//	action(const action& new_action):refActionDef(new_action.refActionDef){
//		name=new_action.name;
//		assigned_agents=new_action.assigned_agents;
//		assignedParameters=new_action.assignedParameters;
//		isDone=new_action.isDone;
//		actionAndParameters=new_action.actionAndParameters;
//		Action_GeneralParameters=new_action.Action_GeneralParameters;
//		GeneralParameters=new_action.GeneralParameters;
//	};
//	action& operator=(const action& new_action){
//
//		refActionDef=new_action.refActionDef;
//		name=new_action.name;
//		assigned_agents=new_action.assigned_agents;
//		assignedParameters=new_action.assignedParameters;
//		isDone=new_action.isDone;
//		actionAndParameters=new_action.actionAndParameters;
//		Action_GeneralParameters=new_action.Action_GeneralParameters;
//		GeneralParameters=new_action.GeneralParameters;
//		return *this;
//	};
//
//	~action(){};
//	void Print(void){
//		cout<<FBLU("+++++++++++++++ action info ++++++++++++++++++")<<endl;
//		cout<<"Action Name: "<<name<<endl;
//
//		cout<<"Assigned Agents: ";
//		for(int i=0; i<assigned_agents.size();i++)
//			cout<<assigned_agents[i]<<" ";
//		cout<<endl;
//
//		cout<<"Assigned parameters: ";
//		for(int i=0; i<assignedParameters.size();i++)
//			cout<<assignedParameters[i]<<" ";
//		cout<<endl;
//
//		cout<<"action with parameters: "<< actionAndParameters<<endl;
//
//		cout<<"Is action done? ";
//		for(int i=0; i<isDone.size();i++)
//			cout<<isDone[i]<<" ";
//		cout<<endl;
//
//		cout<<"Assigned first part of parameters: ";
//		for(int i=0; i<GeneralParameters.size();i++)
//			cout<<GeneralParameters[i]<<" ";
//		cout<<endl;
//
//		cout<<"action with first part of parameters: "<< Action_GeneralParameters<<endl;
//
//		refActionDef.Print();
//		cout<<"++++++++++++++++++++++++++++++++++++++++++++++"<<endl;
//
//	};
//};
//
//
//
////****************************
//class agent{
//public:
//	string name;
//	string type;// Robot or Human
//	string lastAssignedAction;
//	string lastActionAck;
//	bool allowToChangePath;
//	bool isBusy;
//	bool isSuccessfullyDone;
//	int optimal_state;
//	int next_action_index;
//	int responsibility_number;
//
//	agent(void)
//	{
//		name="";
//		type="";
//		lastAssignedAction="";
//		lastActionAck="";
//		allowToChangePath=false;
//		isBusy=false;
//		isSuccessfullyDone=false;
//		responsibility_number=0;
//		optimal_state=0;
//		next_action_index=0;
//	};
//	~agent(){};
//	void Print(void){
//		cout<<FBLU("******** agent info *********")<<endl;
//		cout<<"name: "<<name<<endl;
//		cout<<"type: "<<type<<endl;
//		cout<<"lastAssignedAction: "<<lastAssignedAction<<endl;
//		cout<<"lastActionAck: "<<lastActionAck<<endl;
//		cout<<"allowToChangePath: "<<allowToChangePath<<endl;
//		cout<<"isBusy: "<<isBusy<<endl;
//		cout<<"isSuccessfullyDone? "<<isSuccessfullyDone<<endl;
//	};
//};
//
////****************************
//class offline_state_action{
//public:
//	string state_name;
////	vector<string> actionsList;
////	vector<vector<string>> actionsResponsible;
//
//	vector<action> actions_list;
//
//
//	offline_state_action(void)
//	{
//		state_name="";
//	};
//	~offline_state_action(){};
//	void Print(void){
//		cout<<FBLU("************************* offline_state_action info *********************")<<endl;
//		cout<<"state_name: "<<state_name<<endl;
////		cout<<"actionsList: ";
////		for(int i=0;i<actionsList.size();i++)
////			cout<<actionsList[i]<<" ";
////		cout<<endl;
////		cout<<"Responsible Agents: ";
////		for(int i=0;i<actionsResponsible.size();i++)
////		{
////			for(int j=0;j<actionsResponsible[i].size();j++)
////			{
////				cout<<actionsResponsible[i][j];
////				if(actionsResponsible[i].size()>0)
////					cout<<" ";
////			}
////			cout<<" | ";
////		}
////		cout<<endl;
//
//		cout<<"Actions List: "<<endl;
//		for(int i=0;i<actions_list.size();i++)
//			actions_list[i].Print();
//
//	};
//};
//
////****************************
//class feasible_state_action{
//public:
//	string state_name;
//	string state_type; // node or hyperarc
//	int state_cost;
//	bool isFeasible;
//
//	vector<string> actionsList;
//	vector<vector<string>> actionsResponsible;
//	vector<vector<bool>> actionsProgress;
//
//	vector<action> actions_list;
//	vector<string> stateResponsible;
//	bool isSimulated;
//
//
//	feasible_state_action(void)
//	{
//		state_name="";
//		state_type="";
//		state_cost=0;
//		isFeasible=true;
//		isSimulated=false;
//	};
//	~feasible_state_action(){};
//	void Print(void){
//		cout<<FBLU("********************** feasible state-action table info ***********************")<<endl;
//		cout<<"state_name: "<<state_name<<endl;
//		cout<<"state_type: "<<state_type<<endl;
//		cout<<"state_cost: "<<state_cost<<endl;
//		cout<<"isFeasible: "<<isFeasible<<endl;
//
//		cout<<"actionsList: ";
//		for(int i=0;i<actionsList.size();i++)
//			cout<<actionsList[i]<<" ";
//		cout<<endl;
//
//		cout<<"actionsResponsible: ";
//		for(int i=0;i<actionsResponsible.size();i++)
//		{
//			for(int j=0;j<actionsResponsible[i].size();j++)
//			{
//				cout<<actionsResponsible[i][j];
//				if(actionsResponsible[i].size()>0)
//					cout<<" ";
//			}
//			cout<<" | ";
//		}
//		cout<<endl;
//
//		cout<<"actionsProgress: ";
//		for(int i=0;i<actionsProgress.size();i++)
//		{
//			for(int j=0;j<actionsProgress[i].size();j++)
//			{
//				cout<<actionsProgress[i][j];
//				if(actionsProgress[i].size()>0)
//					cout<<" ";
//			}
//			cout<<" | ";
//		}
//		cout<<endl;
//
//		cout<<"actions list:"<<endl;
//		for(int i=0;i<actions_list.size();i++){
//			actions_list[i].Print();
//		}
//	};
//};
//
////****************************
//// check later if it need a copy constructor and '=' operation;
//class optimal_state_simulation{
//public:
//
//	string state_name;
//
//	vector<action> actions_list;
//	vector<double> actionsTime;
//
//	vector<string> actions_parameters; // [object X/ point X] [object grasping pose] [object Frame (to control)] [goal Frame] [responsible agent]
//	// depending on the action, they use some of these parameters
//	vector<string> parameters_type;
//
//	vector<bool> canAgentsPerformAction;
//
//	vector<string> responsibleAgents;
//	feasible_state_action* optimalStatePtr;
//
//	double total_cost;
//	double simulation_q[2][7];//LeftArm+RightArm;
//	optimal_state_simulation(){
//		state_name="";
//		total_cost=0.0;
//		for(int i=0;i<7;i++){
//		simulation_q[0][i]=0.0;
//		simulation_q[1][i]=0.0;
//		}
//		optimalStatePtr=NULL;
//	};
//
//	~optimal_state_simulation(){};
//	void SetAgentForAllTheAction(void){
//		for(int i=0;i<actions_list.size();i++)
//		{
//			actions_list[i].assigned_agents=responsibleAgents;
//			int NoAgents=responsibleAgents.size();
//			actions_list[i].isDone.resize(NoAgents,false);
//		}
//	};
//
//	void Print(void){
//		cout<<FBLU("*********************** optimal_state_simulation::Print ************************* ")<<endl;
//		cout<<"state name: "<<state_name<<endl;
//		cout<<"Total cost: "<<total_cost<<endl;
//
//		cout<<"actions list: "<<endl;
//		for(int i=0;i<actions_list.size();i++)
//		{
//			actions_list[i].Print();
//			cout<<"action time: "<<actionsTime[i]<<endl;
//
//		}
//	};
//
//
//};

//****************************

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
//    vector<vector<string>> Full_State_action_list;
    vector<vector<offline_state_action>> Full_State_action_list;// list of all the actions for all the states, this list should be found offline by a planner

    vector<feasible_state_action> state_action_table;

    vector<optimal_state_simulation> simulation_vector;
    vector<string> complexAcrionsList;
    bool emergencyFlag;
//    vector<vector<string>> Feasible_states_actions_table;     // table of the feasible state-actions names
//    vector<vector<bool>> Feasible_states_actions_progress;     // table of the feasible state-actions done(true)/not done (false)actions
//    vector<vector<string>> Feasible_states_Names; // the name of the feasible states , type of it :nodes / hyperacs
//    vector<int> Feasible_States_cost; // weight for each feasible state
//    vector<int> Feasible_states_isFeasible;// if still the Feasible given by the and/or graph is feasible (true) or not (false) ?
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
	else{
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
}



