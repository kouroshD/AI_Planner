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



#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST

using namespace std;
class offline_state_action;

class actionDef{
public:
	string name;
	vector<vector<string>> possible_agents; // the agents who "can" perform the action// each raw if the size is one single action, otherwise it is joint action
	string actionType; // if it is simple, or complex (a complex action is another andor graph)
	string actionMode; // if the action should be performed by single agent or jointly between agents
	vector <string> parameterTypes; // what is the feature of an argument we should look for in the knowledge base,
	// for example: if we say: approach plate, in fact we should look for the grasping pose of the plate.
	// it should be given in order
	vector<offline_state_action> ComplexAction_state_action_list;//
	actionDef(void);
	~actionDef();
	void Print(void);
};
//
class action{
public:
	actionDef &refActionDef;
	string name;
	vector<string> assigned_agents;
	vector<string> assignedParameters; // Example: object1-graspingPose1, Point1
	vector<string> GeneralParameters;
	vector<bool> isDone; // it is a vector, because for some joint actions (human+robot) it needs to fill all of them in order to say an action is done;
	string actionAndParameters; //Example:  Approach_object1-graspingPose1, Approach_Point1
	string Action_GeneralParameters;//Example:  Appraoch_object1, Approach_Point1


	action(actionDef &actionDefObj);
	action(const action& new_action);
	action& operator=(const action& new_action);

	~action();
	void Print(void);
};

//****************************
class agent{
public:
	string name;
	string type;// Robot or Human
	string lastAssignedAction;
	string lastActionAck;
	bool allowToChangePath;
	bool isBusy;
	bool isSuccessfullyDone;
	int optimal_state;
	int next_action_index;
	int responsibility_number;

	agent(void);
	~agent();
	void Print(void);
};

//****************************
class offline_state_action{
public:
	string state_name;
//	vector<string> actionsList;
//	vector<vector<string>> actionsResponsible;
	vector<action> actions_list;

	offline_state_action(void);
	~offline_state_action();
	void Print(void);
};

//****************************
class feasible_state_action{
public:
	string state_name;
	string state_type; //! node or hyperarc
	string andorName; //! name of the andor graph the state is belonged to!
	int state_cost;
	bool isFeasible;

	vector<string> actionsList;
	vector<vector<string>> actionsResponsible;
	vector<vector<bool>> actionsProgress;

	vector<action> actions_list;
	vector<string> stateResponsible;
	bool isSimulated;


	feasible_state_action(void);
	~feasible_state_action();
	void Print(void);
};

//****************************
// check later if it need a copy constructor and '=' operation;
class optimal_state_simulation{
public:

	string state_name;

	vector<action> actions_list;
	vector<double> actionsTime;

	vector<string> actions_parameters; // [object X/ point X] [object grasping pose] [object Frame (to control)] [goal Frame] [responsible agent]
	// depending on the action, they use some of these parameters
	vector<string> parameters_type;

	vector<bool> canAgentsPerformAction;

	vector<string> responsibleAgents;
	feasible_state_action* optimalStatePtr;

	double total_cost;
	double simulation_q[2][7];//LeftArm+RightArm;
	optimal_state_simulation();

	~optimal_state_simulation();
	void SetAgentForAllTheAction(void);

	void Print(void);
};
