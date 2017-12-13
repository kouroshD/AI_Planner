#include "seq_planner_class.hpp"

seq_planner_class::seq_planner_class(string actionDefinitionPath,string stateActionPath){
	cout<<"seq_planner_class::seq_planner_class"<<endl;
	optimal_state=0;
	next_action_index=0;
	//	actionList=NULL;

	SetActionDefinitionList(actionDefinitionPath);
	SetStateActionList( stateActionPath);
	SetAgentsList();
	cout<<"****************** Action Definition List *************************"<<endl;
		for(int i=0;i<action_Definition_List.size();i++)
			action_Definition_List[i].Print();
	cout<<"******************* Full State-Action List ************************"<<endl;
		for(int i=0;i<Full_State_action_list.size();i++)
			Full_State_action_list[i].Print();
	cout<<"********************** Agents List*********************"<<endl;
	for(int i=0;i<agents.size();i++)
		agents[i].Print();

	updateAndor=true;
	nodeSolved=false;
	haSolved=false;
	subHumanActionAck=nh.subscribe("HRecAction",100, &seq_planner_class::CallBackHumanAck, this);
	subRobotActionAck=nh.subscribe("robot_ack",100, &seq_planner_class::CallBackRobotAck, this);
	pubRobotCommand = nh.advertise<std_msgs::String>("robot_command",10);

}
seq_planner_class::~seq_planner_class(){
	cout<<"seq_planner_class::~seq_planner_class"<<endl;
}

void seq_planner_class::UpdateStateActionTable(int agent_update){
	cout<<"seq_planner_class::UpdateStateActionTable"<<endl;
	/*! when an acknowledgment arrive from agents:
		 1- Search for action in state-action table
		 2- Update the following state (take the feasible state with minimum cost)
		 	 - if an state is executed inform the andor graph, come out of here
	 */
	cout<<FBLU("before update: ")<<endl;
		for (int i=0;i<state_action_table.size();i++)
			state_action_table[i].Print();

	bool is_a_state_solved=false;
	bool is_a_state_feasible=false;
	if(state_action_table.size()>0)
	{
		for(int i=0;i<state_action_table.size();i++)
		{
			if (state_action_table[i].isFeasible==true)
			{
				bool temp_is_the_state_i_still_feasible=false;
				for (int j=0;j<state_action_table[i].actionsList.size();j++)
				{
					if(j==0)
					{
						if(state_action_table[i].actionsList[j]==agents[agent_update].lastActionAck
								&& state_action_table[i].actionsProgress[j]==false)
						{
							if(agents[agent_update].isSuccessfullyDone==true)
							{
								state_action_table[i].actionsProgress[j]=true;
								temp_is_the_state_i_still_feasible=true;
							}
							break;
						}

					}
					else
					{
						// j>0: check for prev is solved and current is not solved:
						if(state_action_table[i].actionsList[j]==agents[agent_update].lastActionAck
								&& state_action_table[i].actionsProgress[j]==false
								&& state_action_table[i].actionsProgress[j-1]==true)
						{
							if(agents[agent_update].isSuccessfullyDone==true)
							{
								state_action_table[i].actionsProgress[j]=true;
								temp_is_the_state_i_still_feasible=true;
							}
							break;
						}
					}

				}
				if(temp_is_the_state_i_still_feasible==true)
				{
					state_action_table[i].isFeasible=true;
					is_a_state_feasible=true;
				}
				else
				{
					state_action_table[i].isFeasible=false;
				}

			}
			if(state_action_table[i].isFeasible==true
					&& state_action_table[i].actionsProgress[state_action_table[i].actionsProgress.size()-1]==true)
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
				cout<<FBLU("after update: ")<<endl;
				for (int i=0;i<state_action_table.size();i++)
					state_action_table[i].Print();

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
	for(int i=0;i<state_action_table.size();i++)
	{
		if(state_action_table[i].isFeasible==true)
		{
			number_feasible_state++;
			if (state_action_table[i].state_cost<state_min_cost){
				state_min_cost=state_action_table[i].state_cost;
				optimal_state=i;
			}
		}
	}
	if (number_feasible_state>0)
	{
		for(int i=0;i<state_action_table[optimal_state].actionsList.size();i++)
		{
			if(state_action_table[optimal_state].actionsProgress[i]==false)
			{
				next_action_index=i;
				break;
			}
		}
		cout<<"Optimal State: "<<state_action_table[optimal_state].state_name<<", Next Action: "<<state_action_table[optimal_state].actionsList[next_action_index]<<endl;
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

	bool isResponsibleAgentAcceptable=false;
	int action_number=0;
	bool action_is_defined=false;
	for(int i=0;i<action_Definition_List.size();i++)
		if(state_action_table[optimal_state].actionsList[next_action_index]==action_Definition_List[i].name)
		{
			action_number=i;
			action_is_defined=true;
			break;
		}
	if(action_is_defined==false)
	{
		cout<<BOLD(FRED("This Action is not defined in Actions Definition List"))<<state_action_table[optimal_state].actionsList[next_action_index] <<endl;
		exit(1);
	}

	if(state_action_table[optimal_state].actionsResponsible[next_action_index]=="Unknown")
	{
		isResponsibleAgentAcceptable=true;
//		if(action_Definition_List[action_number].agents.size()==1)
//		{
		if(action_Definition_List[action_number].agents[0].size()==1)
			state_action_table[optimal_state].actionsResponsible[next_action_index]=action_Definition_List[action_number].agents[0][0];
		else
			state_action_table[optimal_state].actionsResponsible[next_action_index]="Joint";
		//		}
//		else
//		{
//			//					for (int j=0;j<action_Definition_List[i].agents.size();j++)
//			//					{
//			state_action_table[optimal_state].actionsResponsible[next_action_index]=action_Definition_List[action_number].agents[0][0];
//			//					}
//		}
	}
	else
	{
		//				for (int j=0;j<action_Definition_List[action_number].agents.size();j++)
		//					if (Feasible_State_Action_Table[optimal_state].actionsResponsible[next_action_index]==action_Definition_List[action_number].agents[j])
		isResponsibleAgentAcceptable=true;
	}
// give the command for the find found agent:
	if(isResponsibleAgentAcceptable==true)
	{
		if(state_action_table[optimal_state].actionsResponsible[next_action_index]=="All")
		{
			for(int i=0;i<action_Definition_List[action_number].agents.size();i++)
			{
				if (action_Definition_List[action_number].agents[i].size()==1)
				{
					for(int j=0;j<agents.size();j++)
					{
						if(action_Definition_List[action_number].agents[i][0]==agents[j].name)
						{
							agents[j].lastAssignedAction=state_action_table[optimal_state].actionsList[next_action_index];
							agents[j].isBusy=true;
							if(agents[j].type=="Human")
							{
								PublishHumanAction( agents[j].lastAssignedAction , agents[j].name);
							}
							else if(agents[j].type=="Robot")
							{
								PublishRobotAction( agents[j].lastAssignedAction , agents[j].name);
							}
							else
							{
								cout<<FRED("agent type is wrong")<<endl;
							}
							break;
						}
					}


				}
				else
				{
					//					action_Definition_List[action_number].agents[i].size()>1
					cout<<FRED("This action can not be type 'ALL' : ")<<action_Definition_List[action_number].name<<endl;
				}
			}
		}
		else if(state_action_table[optimal_state].actionsResponsible[next_action_index]=="Joint")
		{
			for(int i=0;i<action_Definition_List[action_number].agents[0].size();i++)
			{
				for(int j=0;j<agents.size();j++)
				{
					if(action_Definition_List[action_number].agents[0][i]==agents[j].name)
					{
						agents[j].lastAssignedAction=state_action_table[optimal_state].actionsList[next_action_index];
						agents[j].isBusy=true;
						break;
					}
				}
			}
			PublishRobotAction(state_action_table[optimal_state].actionsList[next_action_index],
					state_action_table[optimal_state].actionsResponsible[next_action_index]);
		}
		else
		{
			for(int i=0;i<agents.size();i++)
			{
				if(state_action_table[optimal_state].actionsResponsible[next_action_index]==agents[i].name)
				{
					agents[i].lastAssignedAction=state_action_table[optimal_state].actionsList[next_action_index];
					agents[i].isBusy=true;
					if(agents[i].type=="Human")
					{
						PublishHumanAction( agents[i].lastAssignedAction , agents[i].name);
					}
					else if(agents[i].type=="Robot")
					{
						PublishRobotAction( agents[i].lastAssignedAction , agents[i].name);
					}
					else
					{
						cout<<FRED("agent type is wrong")<<endl;
					}
				}

			}
		}

	}
//		if(state_action_table[optimal_state].actionsResponsible[next_action_index]=="Human")
//		{
//			agents[0].lastAssignedAction=state_action_table[optimal_state].actionsList[next_action_index];
//			agents[0].isBusy=true;
//			PublishHumanAction( agents[0].lastAssignedAction , agents[0].name);
//		}
//		else if(state_action_table[optimal_state].actionsResponsible[next_action_index]=="LeftArm")
//		{
//			agents[1].lastAssignedAction=state_action_table[optimal_state].actionsList[next_action_index];
//			agents[1].isBusy=true;
//			PublishRobotAction(agents[1].lastAssignedAction , agents[1].name);
//
//		}
//		else if(state_action_table[optimal_state].actionsResponsible[next_action_index]=="RightArm")
//		{
//			agents[2].lastAssignedAction=state_action_table[optimal_state].actionsList[next_action_index];
//			agents[2].isBusy=true;
////			PublishRobotActionRightArm();
//		}
//		else if(state_action_table[optimal_state].actionsResponsible[next_action_index]=="Joint")
//		{
//			agents[1].lastAssignedAction=state_action_table[optimal_state].actionsList[next_action_index];
//			agents[1].isBusy=true;
//			agents[2].lastAssignedAction=state_action_table[optimal_state].actionsList[next_action_index];
//			agents[2].isBusy=true;
////			PublishRobotActionJointly();
//
//		}
//		else if(state_action_table[optimal_state].actionsResponsible[next_action_index]=="All")
//		{
//			for(int i=0;i<action_Definition_List[action_number].agents.size();i++)
//			{
//				// check better later, it should check for the agents name
//				agents[i].lastAssignedAction=state_action_table[optimal_state].actionsList[next_action_index];
//				agents[i].isBusy=true;
//			}
//			PublishRobotActionLeftArm();
//			PublishRobotActionRightArm();
//		}
	else
	{
		cout<<FRED("No Responsible Agent")<<endl;
	}
//				for(int j=0;j<action_Definition_List[i].agents.size();j++)
//				{
//				// here	should call another function to check the cost of performing the action by each agent.
//				}
//				if(action_Definition_List[i].agents[0]=="Human")
//				{
//					agents[0].lastAssignedAction=action_Definition_List[i].name;
//					PublishHumanAction();
//					agents[0].isBusy=true;
//				}
//				if(action_Definition_List[i].agents[0]=="LeftArm")
//				{
//					agents[1].lastAssignedAction=action_Definition_List[i].name;
//					agents[1].isBusy=true;
//					PublishRobotActionLeftArm();
//				}
//				if(action_Definition_List[i].agents[0]=="RightArm")
//				{
//					agents[2].lastAssignedAction=action_Definition_List[i].name;
//					agents[2].isBusy=true;
//					PublishRobotActionRightArm();
//				}
//				if(action_Definition_List[i].actionMode=="joint")
//				{
//					agents[1].lastAssignedAction=action_Definition_List[i].name;
//					agents[1].isBusy=true;
//					agents[2].lastAssignedAction=action_Definition_List[i].name;
//					agents[2].isBusy=true;
//					PublishRobotActionJointly();
//
//				}
//			}
//			if(action_Definition_List[i].actionMode=="joint")
//			{
//				agents[1].lastAssignedAction=action_Definition_List[i].name;
//				agents[1].isBusy=true;
//				agents[2].lastAssignedAction=action_Definition_List[i].name;
//				agents[2].isBusy=true;
//				PublishRobotActionJointly();
//
//			}
//			if(action_Definition_List[i].actionMode=="all")
//			{
//				agents[1].lastAssignedAction=action_Definition_List[i].name;
//				agents[1].isBusy=true;
//				agents[2].lastAssignedAction=action_Definition_List[i].name;
//				agents[2].isBusy=true;
//				PublishRobotActionLeftArm();
//				PublishRobotActionRightArm();
//			}
//		}
//	}
}

void seq_planner_class::GenerateStateActionTable(vector<vector<string>> gen_Feasible_state_list, vector<int> gen_Feasible_stateCost_list){
	cout<<"seq_planner_class::GenerateStateActionTable"<<endl;
	if(state_action_table.size()>0)
		state_action_table.clear();


//	Feasible_states_Names.clear();
//	Feasible_States_cost.clear();
//	Feasible_states_actions_table.clear();
//	Feasible_states_actions_progress.clear();
//	Feasible_states_isFeasible.clear();
	cout<<"100: "<<gen_Feasible_stateCost_list.size()<<endl;
	for(int i=0; i<gen_Feasible_stateCost_list.size();i++)
	{
		feasible_state_action temp_obj;
		temp_obj.state_name=gen_Feasible_state_list[i][0];
		temp_obj.state_type=gen_Feasible_state_list[i][1];
		temp_obj.state_cost=gen_Feasible_stateCost_list[i];
		temp_obj.isFeasible=true;
		//		temp_obj.actionsList=
			for (int j=0;j<Full_State_action_list.size();j++)
			{
				if (temp_obj.state_name==Full_State_action_list[j].state_name)
				{
					temp_obj.actionsList=Full_State_action_list[j].actionsList;
					temp_obj.actionsResponsible=Full_State_action_list[j].actionsResponsible;
					for (int k=0;k<Full_State_action_list[j].actionsList.size();k++) // check if k should start from 0 or 1 ?
						temp_obj.actionsProgress.push_back(false);
				}
			}
			state_action_table.push_back(temp_obj);
	}

	cout<<"101: "<<state_action_table.size()<<endl;
	for (int i=0;i<state_action_table.size();i++)
		state_action_table[i].Print();
//exit(0);
//	Print2dVec(Feasible_states_Names);
//	Print2dVec(Feasible_States_cost);
//	Print2dVec(Feasible_states_actions_progress);
//	Print2dVec(Feasible_states_actions_table);



	CheckStateExecution();

}
void seq_planner_class::CheckStateExecution(){
	// check for all the actions in all rows of state-action table:
	// if a row is empty OR if all the actions row is done (true flag):
	//that state is solved, Delete all the vector, u
	// if there is not update for the andor graph, find the next action for the human or robot to be solved
	cout<<"seq_planner_class::CheckStateExecution"<<endl;

	if(!Solved_node_list.empty() || !Solved_hyperarc_list.empty()){
		cout<<FRED("The solve nodes or hyperarc lists are not empty!" )<<endl;
	}
	if(state_action_table.size()==0){
		cout<<FRED("There is no feasible state to be checked" )<<endl;
		exit(1);
	}


	for(int i=0;i<state_action_table.size();i++)
	{
		if (state_action_table[i].actionsProgress.size()==0)
		{
			updateAndor=true;
			if (state_action_table[i].state_type=="Node")
			{
				Solved_node_list.push_back(state_action_table[i].state_name);
				nodeSolved=true;
			}
			if (state_action_table[i].state_type=="Hyperarc")
			{
				Solved_hyperarc_list.push_back(state_action_table[i].state_name);
				haSolved=true;
			}
		}
		else
		{
			int actions_size=state_action_table[i].actionsProgress.size();
			if (state_action_table[i].actionsProgress[actions_size-1]==true)
			{
				updateAndor=true;
				if (state_action_table[i].state_type=="Node")
				{
					Solved_node_list.push_back(state_action_table[i].state_name);
					nodeSolved=true;
				}
				if (state_action_table[i].state_type=="Hyperarc")
				{
					Solved_hyperarc_list.push_back(state_action_table[i].state_name);
					haSolved=true;
				}
			}
		}
	}
	if(updateAndor==false)
	{
		FindNextAction();
	}
}

void seq_planner_class::SetActionDefinitionList(string actionDefinitionPath){

		cout<<"seq_planner_class::SetActionDefinitionList"<<endl;
	//	ROS_INFO("%s",actionDefinitionPath.c_str());
	ifstream file_path_ifStr(actionDefinitionPath.c_str());
	string delim_type=" ";
	string delim_agent="/";
	string delim_joint="+";
	std::vector<std::string> line_list;
	string line;

	    cout<<file_path_ifStr.is_open()<<endl;
	if (file_path_ifStr.is_open()){
		while(getline(file_path_ifStr,line))
		{
			action actionDef;
			boost::split(line_list, line, boost::is_any_of(delim_type));
			actionDef.name=line_list[0];
			actionDef.actionType=line_list[1];
//			actionDef.actionMode=line_list[2];
			int counter=0;
//			int Num_agents=stoi(line_list[3]);
			vector<string> AGENTS, joint_agents;
			boost::split(AGENTS, line_list[2], boost::is_any_of(delim_agent));
			for (int i=0;i < AGENTS.size();i++)
			{
				boost::split(joint_agents, AGENTS[i], boost::is_any_of(delim_joint));
//				for (int j=0;j < joint_agents.size();j++)
				actionDef.agents.push_back(joint_agents);
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
	cout<<FRED("seq_planner_class::SetAgentsList")<<endl;
	//agent[0]: human
	agent tempAgent0;
	tempAgent0.allowToChangePath=true;
	tempAgent0.name="Human";
	tempAgent0.type="Human";

	agents.push_back(tempAgent0);

	//agent[1]: LeftArm
	agent tempAgent1;
	tempAgent1.name="LeftArm";
	tempAgent1.type="Robot";
	agents.push_back(tempAgent1);

	//agent[2]: RightArm
	agent tempAgent2;
	tempAgent2.name="RightArm";
	tempAgent2.type="Robot";
	agents.push_back(tempAgent2);

	for(int k=0;k<agents.size();k++)
		agents[k].Print();

}

void seq_planner_class::SetStateActionList(string stateActionPath){
	cout<<"seq_planner_class::SetStateActionList"<<endl;
//	cout<<stateActionPath<<endl;

	cout<<"SetStateActionList"<<endl;
	ifstream file_path_ifStr(stateActionPath.c_str());
	vector<string> line_list;
	vector<string> action_responsible;
	string line;
	string delim_type=" ";
	string responsible_delim_type="_";


	cout<<"file_path_ifStr.is_open(): "<<file_path_ifStr.is_open()<<endl;
	if (file_path_ifStr.is_open())
	{
		while(getline(file_path_ifStr,line))
		{
			offline_state_action temp_obj;
			boost::split(line_list, line, boost::is_any_of(delim_type));
//			Full_State_action_list.push_back(line_list);
			temp_obj.state_name=line_list[0];
			for(int i=1;i<line_list.size();i++)
			{
				boost::split(action_responsible, line_list[i], boost::is_any_of(responsible_delim_type));

				temp_obj.actionsList.push_back(action_responsible[0]);
				if(action_responsible.size()==2)
					temp_obj.actionsResponsible.push_back(action_responsible[1]);
				else if(action_responsible.size()==1)
				{
					temp_obj.actionsResponsible.push_back("Unknown");
				}
				else
				{cout<<"Error in state_action text file"<<endl;}
			}
			Full_State_action_list.push_back(temp_obj);
		}
		file_path_ifStr.close();
	}

//	for (int m=0;m<Full_State_action_list.size();m++){
//		for (int n=0;n<Full_State_action_list[m].size();n++)
//			cout<<Full_State_action_list[m][n]<<" ";
//		cout<<endl;
//	}
//	Print2dVec(Full_State_action_list);
}

void seq_planner_class::CallBackHumanAck(const std_msgs::String::ConstPtr& msg){
	cout<<"seq_planner_class::CallBackHumanAck"<<endl;

	string recognized_human_action=msg-> data.c_str();
	agents[0].isBusy=false;
	agents[0].lastActionAck=recognized_human_action;
	agents[0].isSuccessfullyDone=true;
	int agent_update=0;
	UpdateStateActionTable(agent_update);
}

void seq_planner_class::CallBackRobotAck(const std_msgs::String::ConstPtr& msg){
	cout<<"seq_planner_class::CallBackRobotAck"<<endl;

	ROS_INFO("I heard Robot Ack: [%s]", msg->data.c_str());
	string robot_action_ack;
	robot_action_ack=msg->data.c_str();
	int agent_update;

	if (robot_action_ack=="GoalReachedLeft")
	{
		agents[1].isBusy=false;
		agents[1].lastActionAck=agents[1].lastAssignedAction;
		agents[1].isSuccessfullyDone=true;
		agent_update=1;
		UpdateStateActionTable(agent_update);
	}
	else if (robot_action_ack=="GoalNotReachedLeft")
	{
		agents[1].isBusy=false;
		agents[1].lastActionAck=agents[1].lastAssignedAction;
		agents[1].isSuccessfullyDone=false;
		agent_update=1;
		UpdateStateActionTable(agent_update);
	}
	else if (robot_action_ack=="GoalReachedRight")
	{
		agents[2].isBusy=false;
		agents[2].lastActionAck=agents[2].lastAssignedAction;
		agents[2].isSuccessfullyDone=true;
		agent_update=2;
		UpdateStateActionTable(agent_update);
	}
	else if (robot_action_ack=="GoalNotReachedRight")
	{
		agents[2].isBusy=false;
		agents[2].lastActionAck=agents[2].lastAssignedAction;
		agents[2].isSuccessfullyDone=false;
		agent_update=2;
		UpdateStateActionTable(agent_update);
	}
	else if (robot_action_ack=="GoalReachedJoint")
	{
		agents[1].isBusy=false;
		agents[1].lastActionAck=agents[1].lastAssignedAction;
		agents[1].isSuccessfullyDone=true;
		agents[2].isBusy=false;
		agents[2].lastActionAck=agents[2].lastAssignedAction;
		agents[2].isSuccessfullyDone=true;
		agent_update=1; // in this case we will update both representation later
		UpdateStateActionTable(agent_update);
	}
	else if (robot_action_ack=="GoalNotReachedBiManual")
	{
		agents[1].isBusy=false;
		agents[1].lastActionAck=agents[1].lastAssignedAction;
		agents[1].isSuccessfullyDone=false;
		agents[2].isBusy=false;
		agents[2].lastActionAck=agents[2].lastAssignedAction;
		agents[2].isSuccessfullyDone=false;
		agent_update=1; // in this case we will update both representation later
		UpdateStateActionTable(agent_update);
	}
	else if (robot_action_ack=="RobotTaskReached")
	{// maybe can be commented!

		agents[1].isBusy=false;
		agents[1].lastActionAck=agents[1].lastAssignedAction;
		agents[1].isSuccessfullyDone=true;
		agents[2].isBusy=false;
		agents[2].lastActionAck=agents[2].lastAssignedAction;
		agents[2].isSuccessfullyDone=true;
		agent_update=1;
		UpdateStateActionTable(agent_update);
	}
	else if (robot_action_ack=="RobotTaskNotReached")
	{// maybe can be commented!

		agents[1].isBusy=false;
		agents[1].lastActionAck=agents[1].lastAssignedAction;
		agents[1].isSuccessfullyDone=false;
		agents[2].isBusy=false;
		agents[2].lastActionAck=agents[2].lastAssignedAction;
		agents[2].isSuccessfullyDone=false;
		agent_update=1;
		UpdateStateActionTable(agent_update);
	}
	else
	{
		cout<<FRED("The arrived msg from robot is not defined here")<<endl;
	}
}

void seq_planner_class::PublishHumanAction(string humanAction, string responsibeAgentName){
	cout<<"seq_planner_class::PublishHumanAction"<<endl;
	cout<<responsibeAgentName<<FBLU(" Please perform Action: ")<<humanAction<<endl;
//	agents[0].isBusy=true;
}
void seq_planner_class::PublishRobotAction(string robotAction, string responsibeAgentName){
	cout<<"seq_planner_class::PublishRobotAction"<<endl;
	// publish which agent to perform an action
	std_msgs::String robotMsg;
	robotMsg.data =robotAction+"_"+responsibeAgentName;
	pubRobotCommand.publish(robotMsg);

	ROS_INFO("publish robot command : %s ",robotMsg.data.c_str() );
}
