#include "seq_planner_class.hpp"

seq_planner_class::seq_planner_class(string actionDefinitionPath,string stateActionPath){
	cout<<"seq_planner_class::seq_planner_class"<<endl;
	optimal_state=0;
	next_action_index=0;
	//	actionList=NULL;

	SetActionDefinitionList(actionDefinitionPath);
	SetStateActionList( stateActionPath);
	updateAndor=true;
	nodeSolved=false;
	haSolved=false;
	subHumanActionAck=nh.subscribe("HRecAction",100, &seq_planner_class::CallBackHumanAck, this);
	subRobotActionAck=nh.subscribe("HRecAction",100, &seq_planner_class::CallBackRobotAck, this);

}
seq_planner_class::~seq_planner_class(){
	cout<<"seq_planner_class::~seq_planner_class"<<endl;
}

void seq_planner_class::UpdateStateActionTable(void){
	cout<<"seq_planner_class::UpdateStateActionTable"<<endl;
	/*! when an acknowledgment arrive from agents:
		 1- Search for action in state-action table
		 2- Update the following state (take the feasible state with minimum cost)
		 	 - if an state is executed inform the andor graph, come out of here
	 */

	bool is_a_state_solved=false;
	bool is_a_state_feasible=false;
	if(Feasible_states_actions_table.size()>0)
	{
		for(int i=0;i<Feasible_states_actions_table.size();i++)
		{
			if (Feasible_states_isFeasible[i]==true)
			{
				bool temp_is_the_state_i_still_feasible=false;
				for (int j=0;j<Feasible_states_actions_table[i].size();j++)
				{
					if(j==0)
					{
						if(Feasible_states_actions_table[i][j]==agents[agent_update].lastActionAck
								&& Feasible_states_actions_progress[i][j]==false)
						{
							if(agents[agent_update].isSuccessfullyDone==true)
							{
								Feasible_states_actions_progress[i][j]=true;
								temp_is_the_state_i_still_feasible=true;
							}
						}

					}
					else
					{
						// j>0: check for prev is solved and current is not solved:
						if(Feasible_states_actions_table[i][j]==agents[agent_update].lastActionAck
								&& Feasible_states_actions_progress[i][j]==false
								&& Feasible_states_actions_progress[i][j-1]==true)
						{
							if(agents[agent_update].isSuccessfullyDone==true)
							{
								Feasible_states_actions_progress[i][j]=true;
								temp_is_the_state_i_still_feasible=true;
							}
						}


					}

				}
				if(temp_is_the_state_i_still_feasible==true)
				{
					Feasible_states_isFeasible[i]=true;
					is_a_state_feasible=true;
				}
				else
				{
					Feasible_states_isFeasible[i]=false;
				}

			}
			if(Feasible_states_isFeasible[i]==true
					&& Feasible_states_actions_progress[i][Feasible_states_actions_table[i].size()-1]==true)
			{
				CheckStateExecution();
				is_a_state_solved=true;
				break;
			}
		}

		if(is_a_state_solved==false)
		{
			if(is_a_state_feasible==true)
			{
				FindNextAction();
			}
			else
			{
				cout<<FRED("Error, There is no Feasible state now")<<endl;
				exit(1);
			}
		}
	}
	else
	{
		cout<<FRED("Error, the state action table is Empty")<<endl;
		exit(1);
	}
}

void seq_planner_class::FindNextAction(){
	cout<<"seq_planner_class::FindNextAction"<<endl;
	/*!
	 * 	1- Find next action to perform
 	 *	2- Assign an agent to perform the action
 	 */

	int state_min_cost=10000; // a high value
	int number_feasible_state=0;
	for(int i=0;i<Feasible_states_isFeasible.size();i++)
	{
		if(Feasible_states_isFeasible[i]==true)
		{
			number_feasible_state++;
			if (Feasible_States_cost[i]<state_min_cost){
				state_min_cost=Feasible_States_cost[i];
				optimal_state=i;
			}
		}
	}
	if (number_feasible_state>0)
	{
		for(int i=0;i<Feasible_states_actions_table[optimal_state].size();i++)
		{
			if(Feasible_states_actions_progress[optimal_state][i]==false)
			{
				next_action_index=i;
				break;
			}
		}
		FindResponisibleAgent();
	}
	else
	{
		cout<<FRED("Error, There is no Feasible state now")<<endl;
		exit(1);
	}
}

void seq_planner_class::FindResponisibleAgent(void){
	cout<<"seq_planner_class::FindResponisibleAgent"<<endl;
	for(int i=0;i<action_Definition_List.size();i++)
	{
		if(Feasible_states_actions_table[optimal_state][next_action_index]==action_Definition_List[i].name)
		{
			if(action_Definition_List[i].actionMode=="single")
			{
//				for(int j=0;j<action_Definition_List[i].agents.size();j++)
//				{
//				// here	should call another function to check the cost of performing the action by each agent.
//				}
				if(action_Definition_List[i].agents[0]=="Human")
				{
					agents[0].lastAssignedAction=action_Definition_List[i].name;
					PublishHumanAction();
					agents[0].isBusy=true;
				}
				if(action_Definition_List[i].agents[0]=="LeftArm")
				{
					agents[1].lastAssignedAction=action_Definition_List[i].name;
					agents[1].isBusy=true;
					PublishRobotActionLeftArm();
				}
				if(action_Definition_List[i].agents[0]=="RightArm")
				{
					agents[2].lastAssignedAction=action_Definition_List[i].name;
					agents[2].isBusy=true;
					PublishRobotActionRightArm();
				}
			}
			if(action_Definition_List[i].actionMode=="joint")
			{
				agents[1].lastAssignedAction=action_Definition_List[i].name;
				agents[1].isBusy=true;
				agents[2].lastAssignedAction=action_Definition_List[i].name;
				agents[2].isBusy=true;
				PublishRobotActionJointly();

			}
			if(action_Definition_List[i].actionMode=="all")
			{
				agents[1].lastAssignedAction=action_Definition_List[i].name;
				agents[1].isBusy=true;
				agents[2].lastAssignedAction=action_Definition_List[i].name;
				agents[2].isBusy=true;
				PublishRobotActionLeftArm();
				PublishRobotActionRightArm();


			}
		}
	}

}

void seq_planner_class::GenerateStateActionTable(vector<vector<string>> gen_Feasible_state_list, vector<int> gen_Feasible_stateCost_list){
	cout<<"seq_planner_class::GenerateStateActionTable"<<endl;
	Feasible_states_Names.clear();
	Feasible_States_cost.clear();
	Feasible_states_actions_table.clear();
	Feasible_states_actions_progress.clear();
	Feasible_states_isFeasible.clear();

	Feasible_states_Names=gen_Feasible_state_list;
	Feasible_States_cost=gen_Feasible_stateCost_list;


	for (int i=0;i<Feasible_States_cost.size();i++)
	{
		Feasible_states_isFeasible.push_back(true);
		for (int j=0;j<Full_State_action_list.size();j++)
		{

			if (Feasible_states_Names[i][0]==Full_State_action_list[j][0])
			{
				vector<string> Feasible_State_action_row;
				vector<bool> Feasible_states_actions_progress_row;

				for (int k=1;k<Full_State_action_list[j].size();k++) // check if k should start from 0 or 1 ?
				{
					Feasible_State_action_row.push_back(Full_State_action_list[j][k]);
					Feasible_states_actions_progress_row.push_back(false);

				}
				Feasible_states_actions_table.push_back(Feasible_State_action_row);
				Feasible_states_actions_progress.push_back(Feasible_states_actions_progress_row);
			}
		}
	}
	Print2dVec(Feasible_states_Names);
	Print2dVec(Feasible_States_cost);
	Print2dVec(Feasible_states_actions_progress);
	Print2dVec(Feasible_states_actions_table);



	CheckStateExecution();

}
void seq_planner_class::CheckStateExecution(){
	// check for all the actions in all rows of state-action table:
	// if a row is empty OR if all the actions row is done (true flag):
	//that state is solved, Delete all the vector, u
	cout<<"seq_planner_class::CheckStateExecution"<<endl;
	cout<<301<<endl;
	if(!Solved_node_list.empty() || !Solved_hyperarc_list.empty()){
		cout<<302<<endl;
		cout<<FRED("The solve nodes or hyperarc lists are not empty!" )<<endl;
	}

	cout<<Feasible_states_actions_progress.size()<<endl;
	for(int i=0;i<Feasible_states_actions_progress.size();i++)
	{
		cout<<303<<endl;
		if (Feasible_states_actions_progress[i].size()==0)
		{
			cout<<304<<endl;
			updateAndor=true;
			if (Feasible_states_Names[i][1]=="Node")
			{
				cout<<305<<endl;
				Solved_node_list.push_back(Feasible_states_Names[i][0]);
				nodeSolved=true;
				cout<<3052<<endl;
			}
			if (Feasible_states_Names[i][1]=="Hyperarc")
			{
				cout<<306<<endl;
				Solved_hyperarc_list.push_back(Feasible_states_Names[i][0]);
				haSolved=true;
			}
		}
		else
		{
			cout<<307<<endl;
			int counter=0;
			for(int j=0;j<Feasible_states_actions_progress[i].size();j++)
			{
				cout<<308<<endl;

				if (Feasible_states_actions_progress[i][j]==true)
				{
					cout<<309<<endl;
					counter++;
				}
			}
			if (Feasible_states_actions_progress[i].size()==counter)
			{
				cout<<310<<endl;
				updateAndor=true;
				if (Feasible_states_Names[i][1]=="Node")
				{
					cout<<311<<endl;
					Solved_node_list.push_back(Feasible_states_Names[i][0]);
					nodeSolved=true;
				}
				if (Feasible_states_Names[i][1]=="Hyperarc")
				{
					cout<<312<<endl;
					Solved_hyperarc_list.push_back(Feasible_states_Names[i][0]);
					haSolved=true;
				}
			}
		}
	}
	cout<<updateAndor<<endl;
	cout<<nodeSolved<<endl;
	cout<<haSolved<<endl;
}

void seq_planner_class::SetActionDefinitionList(string actionDefinitionPath){

		cout<<"seq_planner_class::SetActionDefinitionList"<<endl;
	//	ROS_INFO("%s",actionDefinitionPath.c_str());
	ifstream file_path_ifStr(actionDefinitionPath.c_str());
	string delim_type=" ";
	std::vector<std::string> line_list;
	string line;
	action actionDef;
	    cout<<file_path_ifStr.is_open()<<endl;
	if (file_path_ifStr.is_open()){
		while(getline(file_path_ifStr,line)){
			boost::split(line_list, line, boost::is_any_of(delim_type));
			actionDef.name=line_list[0];
			actionDef.actionType=line_list[1];
			actionDef.actionMode=line_list[2];
			int counter=0;
			int Num_agents=stoi(line_list[3]);
			for (int i=0;i < Num_agents;i++)
			{
				actionDef.agents.push_back(line_list[4+i]);
				counter++;
			}
			action_Definition_List.push_back(actionDef);
		}
		file_path_ifStr.close();
	}

	//    for (int m=0;m<actionList.size();m++){
	//    	cout<<actionList[m].name<<" "<<actionList[m].actionType<<" "<<actionList[m].actionMode<<endl;
	//    }
}
void seq_planner_class::SetAgentsList(void){
	cout<<"seq_planner_class::SetAgentsList"<<endl;
	//agent[0]: human
	agent tempAgent0;
	tempAgent0.allowToChangePath=true;
	tempAgent0.name="Human";
	agents.push_back(tempAgent0);
	tempAgent0.~agent();

	//agent[1]: LeftArm
	agent tempAgent1;
	tempAgent1.name="LeftArm";
	agents.push_back(tempAgent1);
	tempAgent1.~agent();

	//agent[2]: RightArm
	agent tempAgent2;
	tempAgent2.name="RightArm";
	agents.push_back(tempAgent2);
	tempAgent2.~agent();
}

void seq_planner_class::SetStateActionList(string stateActionPath){
	cout<<"seq_planner_class::SetStateActionList"<<endl;
//	cout<<stateActionPath<<endl;

	cout<<"SetStateActionList"<<endl;
	ifstream file_path_ifStr(stateActionPath.c_str());
	std::vector<std::string> line_list;
	string line;
	string delim_type=" ";
	cout<<"file_path_ifStr.is_open(): "<<file_path_ifStr.is_open()<<endl;
	if (file_path_ifStr.is_open()){
		while(getline(file_path_ifStr,line)){
			boost::split(line_list, line, boost::is_any_of(delim_type));
			Full_State_action_list.push_back(line_list);
		}
		file_path_ifStr.close();
	}

//	for (int m=0;m<Full_State_action_list.size();m++){
//		for (int n=0;n<Full_State_action_list[m].size();n++)
//			cout<<Full_State_action_list[m][n]<<" ";
//		cout<<endl;
//	}
	Print2dVec(Full_State_action_list);
}

void seq_planner_class::CallBackHumanAck(const std_msgs::String::ConstPtr& msg){
	cout<<"seq_planner_class::CallBackHumanAck"<<endl;

	string recognized_human_action=msg-> data.c_str();
	agents[0].isBusy=false;
	agents[0].lastActionAck=recognized_human_action;
	agents[0].isSuccessfullyDone=true;
	agent_update=0;
}

void seq_planner_class::CallBackRobotAck(const std_msgs::String::ConstPtr& msg){
	cout<<"seq_planner_class::CallBackRobotAck"<<endl;

	ROS_INFO("I heard Robot Ack: [%s]", msg->data.c_str());
	string robot_action_ack;
	robot_action_ack=msg->data.c_str();

	if (robot_action_ack=="GoalReachedLeft")
	{
		agents[1].isBusy=false;
		agents[1].lastActionAck=agents[1].lastAssignedAction;
		agents[0].isSuccessfullyDone=true;
		agent_update=1;
	}
	else if (robot_action_ack=="GoalReachedRight")
	{
		agents[2].isBusy=false;
		agents[2].lastActionAck=agents[2].lastAssignedAction;
		agents[0].isSuccessfullyDone=true;
		agent_update=2;
	}
	else if (robot_action_ack=="GoalReachedBiManual")
	{
		agents[1].isBusy=false;
		agents[1].lastActionAck=agents[1].lastAssignedAction;
		agents[2].isBusy=false;
		agents[2].lastActionAck=agents[2].lastAssignedAction;
		agents[0].isSuccessfullyDone=true;
		agent_update=1; // in this case we will update both representation later
	}
	else if (robot_action_ack=="RobotTaskReached")
	{// maybe can be commented!

		agents[1].isBusy=false;
		agents[1].lastActionAck=agents[1].lastAssignedAction;
		agents[2].isBusy=false;
		agents[2].lastActionAck=agents[2].lastAssignedAction;
		agents[0].isSuccessfullyDone=true;
		agent_update=1;
	}
}

void seq_planner_class::PublishHumanAction(void){
	cout<<"seq_planner_class::PublishHumanAction"<<endl;
	cout<<FBLU("Human Please perform Action: ")<<agents[0].lastAssignedAction<<endl;
//	agents[0].isBusy=true;
}
void seq_planner_class::PublishRobotActionLeftArm(void){
	cout<<"seq_planner_class::PublishRobotActionLeftArm"<<endl;
	// publish which agent to perform an action
	std_msgs::String robotMsg;
	robotMsg.data =agents[1].lastAssignedAction+"_left";
	pubRobotCommand.publish(robotMsg);

}

void seq_planner_class::PublishRobotActionRightArm(void){
	cout<<"seq_planner_class::PublishRobotActionRightArm"<<endl;
	// publish which agent to perform an action
	std_msgs::String robotMsg;
	robotMsg.data =agents[1].lastAssignedAction+"_right";
	pubRobotCommand.publish(robotMsg);

}

void seq_planner_class::PublishRobotActionJointly(void){
	cout<<"seq_planner_class::PublishRobotActionJointly"<<endl;
	// publish which agent to perform an action
	std_msgs::String robotMsg;
	robotMsg.data =agents[1].lastAssignedAction+"_joint";
	pubRobotCommand.publish(robotMsg);

}

