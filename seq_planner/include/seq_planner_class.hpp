#include <iostream>
#include <stdio.h>
#include <ros/ros.h>

#define RST  "\x1B[0m"
#define KBLU  "\x1B[34m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define FBLU(x) KBLU x RST
#define FRED(x) KRED x RST
#define FGRN(x) KGRN x RST
#define BOLD(x) "\x1B[1m" x RST

using namespace std;

class seq_planner_class{
public:

	void SetFeasibleStates(void);



private:

	//! the length of the vector is equal to number of agents, if the agent[i] is responsible is true, otherwise it is false;
	vector<bool> responsible_agent;

	int optimal_state;

	void CallBackHumanAck(void);
	void CallBackRobotAck(void);
	void UpdateActionStateTable(void);




};
