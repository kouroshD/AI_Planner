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
	emergencyFlag=false;

}
seq_planner_class::~seq_planner_class(){
	cout<<"seq_planner_class::~seq_planner_class"<<endl;
}

void seq_planner_class::UpdateStateActionTable(string ActionName, vector<string>AgentsName, bool success){
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
	bool is_an_action_done=false;
	vector<bool> actions_done;

	if(state_action_table.size()>0)
	{
		for(int i=0;i<state_action_table.size();i++)
		{
			actions_done.push_back(false);
			bool temp_is_the_state_i_still_feasible=false;
			if (state_action_table[i].isFeasible==true)
			{


				bool break_flag=false;
				for (int j=0;j<state_action_table[i].actionsList.size();j++)
				{
					//					if(j==0)
					//					{
					for(int k=0;k<state_action_table[i].actionsProgress[j].size();k++)
					{

						if(state_action_table[i].actionsProgress[j][k]==false)
						{// here we know where is the first action not have been done in a state:
							for(int n=0;n<AgentsName.size();n++)
							{
								cout<<"500-0: "<<state_action_table[i].actionsList[j]<<ActionName<<endl;
								if(state_action_table[i].actionsList[j]==ActionName)
								{
									cout<<"500-1: "<<state_action_table[i].actionsList[j]<<ActionName<<endl;
									//&&
									//state_action_table[i].actionsResponsible[j][k]==agents[agent_update].name
									for (int m=0;m<state_action_table[i].actionsResponsible[j].size();m++)
									{
										cout<<"500-2: "<<state_action_table[i].actionsResponsible[j][m]<<AgentsName[n] <<endl;

										if(state_action_table[i].actionsResponsible[j][m]==AgentsName[n])
										{
											cout<<"500-3: "<<state_action_table[i].actionsResponsible[j][m]<<AgentsName[n] <<success<<endl;

											if(success==true)
											{
												cout<<501<<" "<<temp_is_the_state_i_still_feasible<<endl;
												state_action_table[i].actionsProgress[j][m]=true;
												temp_is_the_state_i_still_feasible=true;
												cout<<502<<" "<<temp_is_the_state_i_still_feasible<<endl;
												break;
											}
										}

										else if(state_action_table[i].actionsResponsible[j][m]=="Unknown")
										{
											cout<<"500-4: "<<state_action_table[i].actionsResponsible[j][m]<<AgentsName[n] <<success<<endl;
											vector<string> agentNameStr;
											agentNameStr.push_back(AgentsName[n]);
											bool agent_can_perfrom_the_action=CanAgentPerformAction(agentNameStr,"",state_action_table[i].actionsList[j],false);
											// if the action an agent performed is a joint action, we should inform other agents to perfrom the action also!
											if(agent_can_perfrom_the_action==true)
											{
												if(success==true)
												{
													cout<<5010<<" "<<temp_is_the_state_i_still_feasible<<endl;
													state_action_table[i].actionsProgress[j][m]=true;
													state_action_table[i].actionsResponsible[j][m]=AgentsName[n];
													temp_is_the_state_i_still_feasible=true;
													cout<<5020<<" "<<temp_is_the_state_i_still_feasible<<endl;
													break;
												}

											}
											else
											{
												cout<<"The agent: "<<AgentsName[n]<<"could not perform action:"<<state_action_table[i].actionsList[j] <<endl;
											}
										}
										else
										{
											if(state_action_table[i].actionsResponsible[j].size()==1)
											{
												cout<<"The agent: "<<AgentsName[n]<<" performed the action:"<<state_action_table[i].actionsList[j] <<endl;
												cout<<", but the assigned agent to that is: "<< state_action_table[i].actionsResponsible[j][k]<<endl;
												exit(1);
											}
										}
									}

									//							else
									//							{
									//								// if an action performed not successfully by an agent that state will become infeasible
									//								break;
									//							}

									//								for()

								}
								else
								{
									vector<string> emptyStr;
									if(AgentsName[n]!="Human") // later for multiple humans with different names, here should be agent_type instead of agent name
										temp_is_the_state_i_still_feasible=CanAgentPerformAction(emptyStr,"Human",state_action_table[i].actionsList[j], false);

									cout<<503<<" "<<temp_is_the_state_i_still_feasible<<endl;

									break;
								}

								cout<<5030011<<": "<<temp_is_the_state_i_still_feasible<<endl;

							}
							actions_done[i]=true;
							cout<<5030012<<": "<<actions_done[i]<<endl;

							for(int q=0;q<state_action_table[i].actionsProgress[j].size();q++)
							{
								cout<<50300070<<"****: "<<state_action_table[i].actionsList[j]<<state_action_table[i].actionsProgress[j][q]	<<endl;
								if(state_action_table[i].actionsProgress[j][q]==false)
								{
									actions_done[i]=false;
								}

							}
							cout<<5030013<<": "<<actions_done[i]<<endl;

							cout<<5030014<<": "<<temp_is_the_state_i_still_feasible<<endl;
							break_flag=true;
							break; //2 breaks i need
						}
						cout<<5030015<<": "<<temp_is_the_state_i_still_feasible<<endl;
					}

					//					}
					//					else
					//					{
					// j>0: check for prev is solved and current is not solved:
					//						if(state_action_table[i].actionsList[j]==agents[agent_update].lastActionAck
					//								&& state_action_table[i].actionsProgress[j]==false
					//								&& state_action_table[i].actionsProgress[j-1]==true)
					//						{
					//							if(agents[agent_update].isSuccessfullyDone==true)
					//							{
					//								state_action_table[i].actionsProgress[j]=true;
					//								temp_is_the_state_i_still_feasible=true;
					//							}
					//							break;
					//						}
					//					}
					if(break_flag==true){
						cout<<503002<<": "<<temp_is_the_state_i_still_feasible<<endl;
						break;
					}
					cout<<503003<<": "<<temp_is_the_state_i_still_feasible<<endl;

				}
				cout<<503004<<": "<<temp_is_the_state_i_still_feasible<< state_action_table[i].isFeasible<< " "<< state_action_table[i].state_name<<endl;
				if(temp_is_the_state_i_still_feasible==true)
				{
					cout<<504<<" "<<temp_is_the_state_i_still_feasible<<endl;
					state_action_table[i].isFeasible=true;
					is_a_state_feasible=true;
				}
				else
				{
					cout<<505<<" "<<temp_is_the_state_i_still_feasible<<endl;
					state_action_table[i].isFeasible=false;
				}

			}

			if(state_action_table[i].isFeasible==true)
			{
				cout<<506<<" "<<state_action_table[i].isFeasible<<endl;
				bool executionFlag=true;
				for(int n=0;n <state_action_table[i].actionsProgress[state_action_table[i].actionsProgress.size()-1].size(); n++)
					if(state_action_table[i].actionsProgress[state_action_table[i].actionsProgress.size()-1][n]==false)
						executionFlag=false;
				if(executionFlag==true)
				{
					CheckStateExecution();
					is_a_state_solved=true;
					break;
				}
			}
		}


		cout<<"550: "<<is_a_state_solved<<endl;

		for(int h=0;h<actions_done.size();h++)
			if(actions_done[h]==true)
				is_an_action_done=true;

		if(is_a_state_solved==false)
		{
			cout<<"551: "<<is_a_state_solved<<is_a_state_feasible<<is_an_action_done<<endl;
			if(is_a_state_feasible==true)
			{

				cout<<FBLU("after update: ")<<endl;
				for (int i=0;i<state_action_table.size();i++)
					state_action_table[i].Print();
				if(is_an_action_done==true)
				{
					FindNextAction();
				}
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
		bool next_action_progress;
		for(int i=0;i<state_action_table[optimal_state].actionsList.size();i++)
		{
			next_action_progress=false;
			for (int j=0;j<state_action_table[optimal_state].actionsProgress[i].size();j++)
				if (state_action_table[optimal_state].actionsProgress[i][j]==true)
					next_action_progress=true;

			if(next_action_progress==false)
			{
				next_action_index=i;
				break;
			}
		}
		if(next_action_progress==true){
			cout<<"In the optimal state there is no action that all the progress flag is false "<<endl;
			cout<<"You should check later the function"<<endl;
		}
		else
		{
			cout<<"Optimal State: "<<state_action_table[optimal_state].state_name<<", Next Action: "<<state_action_table[optimal_state].actionsList[next_action_index]<<endl;
			FindResponisibleAgent();
		}
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

	if(state_action_table[optimal_state].actionsResponsible[next_action_index][0]=="Unknown")
	{
		isResponsibleAgentAcceptable=true;
		//		if(action_Definition_List[action_number].agents.size()==1)
		//		{
		//		if(action_Definition_List[action_number].agents[0].size()==1)
		vector<bool> action_progress;
		state_action_table[optimal_state].actionsResponsible[next_action_index].clear();
		state_action_table[optimal_state].actionsResponsible[next_action_index]=action_Definition_List[action_number].agents[0];
		for (int i=0;i<state_action_table[optimal_state].actionsResponsible[next_action_index].size();i++)
			action_progress.push_back(false);
		state_action_table[optimal_state].actionsProgress[next_action_index]=action_progress;
		//		else
		//			state_action_table[optimal_state].actionsResponsible[next_action_index]="Joint";
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

		isResponsibleAgentAcceptable=CanAgentPerformAction(state_action_table[optimal_state].actionsResponsible[next_action_index],"",state_action_table[optimal_state].actionsList[next_action_index], true);
	}


	// give the command for the find found agent:
	//	if(isResponsibleAgentAcceptable==true)
	//	{
	//		if(state_action_table[optimal_state].actionsResponsible[next_action_index]=="All")
	//		{
	//			for(int i=0;i<action_Definition_List[action_number].agents.size();i++)
	//			{
	//				if (action_Definition_List[action_number].agents[i].size()==1)
	//				{
	//					for(int j=0;j<agents.size();j++)
	//					{
	//						if(action_Definition_List[action_number].agents[i][0]==agents[j].name)
	//						{
	//							agents[j].lastAssignedAction=state_action_table[optimal_state].actionsList[next_action_index];
	//							agents[j].isBusy=true;
	//							if(agents[j].type=="Human")
	//							{
	//								PublishHumanAction( agents[j].lastAssignedAction , agents[j].name);
	//							}
	//							else if(agents[j].type=="Robot")
	//							{
	//								PublishRobotAction( agents[j].lastAssignedAction , agents[j].name);
	//							}
	//							else
	//							{
	//								cout<<FRED("agent type is wrong")<<endl;
	//							}
	//							break;
	//						}
	//					}
	//
	//
	//				}
	//				else
	//				{
	//					//					action_Definition_List[action_number].agents[i].size()>1
	//					cout<<FRED("This action can not be type 'ALL' : ")<<action_Definition_List[action_number].name<<endl;
	//				}
	//			}
	//		}
	//		else if(state_action_table[optimal_state].actionsResponsible[next_action_index]=="Joint")
	//		{
	//			for(int i=0;i<action_Definition_List[action_number].agents[0].size();i++)
	//			{
	//				for(int j=0;j<agents.size();j++)
	//				{
	//					if(action_Definition_List[action_number].agents[0][i]==agents[j].name)
	//					{
	//						agents[j].lastAssignedAction=state_action_table[optimal_state].actionsList[next_action_index];
	//						agents[j].isBusy=true;
	//						break;
	//					}
	//				}
	//			}
	//			for (int k=0;k<state_action_table[optimal_state].actionsResponsible[next_action_index].size();k++)
	//			PublishRobotAction(state_action_table[optimal_state].actionsList[next_action_index],
	//					state_action_table[optimal_state].actionsResponsible[next_action_index][k]);
	//		}
	//		else
	//		{
	//give the command to the related agent
	if(isResponsibleAgentAcceptable==true)
	{
		bool robotcommandSent=false;
		string action_mode;
		if(state_action_table[optimal_state].actionsResponsible[next_action_index].size()==1)
			action_mode="single";
		else
			action_mode="joint";

		vector<string> robot_agents;
		vector<string> human_colleagues;



		for(int i=0;i<state_action_table[optimal_state].actionsResponsible[next_action_index].size();i++)
		{
			for(int j=0;j<agents.size();j++)
			{
				if(state_action_table[optimal_state].actionsResponsible[next_action_index][i]==agents[j].name)
				{
					vector<string> all_colleagues;
					for(int k=0;k<state_action_table[optimal_state].actionsResponsible[next_action_index].size();k++)
						if(k!=i)
							all_colleagues.push_back(state_action_table[optimal_state].actionsResponsible[next_action_index][k]);

					agents[j].lastAssignedAction=state_action_table[optimal_state].actionsList[next_action_index];
					agents[j].isBusy=true;
					agents[j].responsibility_number=i;

					if(agents[j].type=="Human")
					{
						PublishHumanAction( agents[j].lastAssignedAction , agents[j].name,all_colleagues);
						human_colleagues.push_back(agents[j].name);
					}
					else if(agents[j].type=="Robot")
					{
						robot_agents.push_back(agents[j].name);
					}
					else
					{
						cout<<FRED("agent type is wrong")<<endl;
					}
					break;
				}
			}
			if(i==state_action_table[optimal_state].actionsResponsible[next_action_index].size()-1 && robot_agents.size()>0)
			{
				PublishRobotAction( state_action_table[optimal_state].actionsList[next_action_index], robot_agents,human_colleagues);
			}
		}
	}

	//	}

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
		cout<<FRED("The agent you defined in the 'State-Action-List' file can not perform the given action: state:")<<
				state_action_table[optimal_state].state_name<<", action:"<<state_action_table[optimal_state].actionsList[next_action_index]<<endl;
		cout<<"Do you want to assign a new agent to it? ";
		bool input;
		vector<string> agent_list;
		vector<bool>action_progress;
		cin>>input;
		if(input==true)
		{
			cout<<"Give one of the following rows as responsible:"<<endl;
			for(int m=0;m<action_Definition_List[action_number].agents.size();m++){
				for(int n=0;n<action_Definition_List[action_number].agents[m].size();n++){
					cout<<action_Definition_List[action_number].agents[m][n];
					if(n<action_Definition_List[action_number].agents[m].size()-1)
						cout<<"+";
				}
				cout<<endl;
			}
			string input_string;

			cin>>input_string;
			boost::split(agent_list, input_string, boost::is_any_of("+"));
		}
		else
		{
			agent_list.push_back("Unknown");
		}
		state_action_table[optimal_state].actionsResponsible[next_action_index]=agent_list;
		for (int i=0;i<agent_list.size();i++)
			action_progress.push_back(false);
		state_action_table[optimal_state].actionsProgress[next_action_index]=action_progress;
		FindResponisibleAgent();
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
	bool nameFlag=false;
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
				nameFlag=true;
				temp_obj.actionsList=Full_State_action_list[j].actionsList;
				temp_obj.actionsResponsible=Full_State_action_list[j].actionsResponsible;
				for (int k=0;k<temp_obj.actionsResponsible.size();k++)
				{
					vector<bool> temp_actionProgress;
					for(int l=0;l<temp_obj.actionsResponsible[k].size();l++)
						temp_actionProgress.push_back(false);
					temp_obj.actionsProgress.push_back(temp_actionProgress);
				}
			}
		}
		state_action_table.push_back(temp_obj);
	}
	if(nameFlag==false)
		cout<<"There is not any defined state with the name coming from AND/OR graph"<<endl;

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
			bool last_action_progress=true;

			for (int j=0;j<state_action_table[i].actionsProgress[actions_size-1].size();j++)
				if (state_action_table[i].actionsProgress[actions_size-1][j]==false)
					last_action_progress=false;

			if (last_action_progress==true)
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
	vector<string> action_and_responsibles;

	string line;
	string delim_type=" ";
	string responsible_delim_type="_";
	string joint_delim_type="+";


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
				vector<string> responsibles;
				boost::split(action_and_responsibles, line_list[i], boost::is_any_of(responsible_delim_type));

				temp_obj.actionsList.push_back(action_and_responsibles[0]);
				if(action_and_responsibles.size()==2)
				{
					boost::split(responsibles, action_and_responsibles[1], boost::is_any_of(joint_delim_type));
					temp_obj.actionsResponsible.push_back(responsibles);
				}
				else if(action_and_responsibles.size()==1)
				{
					responsibles.push_back("Unknown");
					temp_obj.actionsResponsible.push_back(responsibles);
				}
				else
				{cout<<"Error in state_action text file, please check state: "<<temp_obj.state_name<<", action: "<<action_and_responsibles[0]<<endl;}
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



bool seq_planner_class::CanAgentPerformAction(vector<string> agent_name, string agent_type, string action_name, bool sufficiency){
	cout<<"seq_planner_class::CanAgentPerformAction"<<endl;
	cout<<"actions_name: "<<action_name<<endl;
	cout<<"agent_type: "<<agent_type<<endl;
	cout<<"agent_name: ";
	for (int i=0;i<agent_name.size();i++)
		cout<<agent_name[i]<<" ";
	cout<<endl;

	bool temp=false;
	int action_number=1000;
	if(agent_name.empty()!=true && agent_type=="")// if a specific agent (by name) can perform an action
	{
		cout<<600<<endl;
		for (int i=0;i<action_Definition_List.size();i++)
		{
			if(action_Definition_List[i].name==action_name)
			{
				action_number=i;
				cout<<601<<": "<<action_number<<endl;
				break;
			}
		}
		if(action_number==1000)
		{
			cout<<"The acition not found in action's list"<<endl;
			exit(1);
		}

		int count;
		for(int j=0;j<action_Definition_List[action_number].agents.size();j++)
		{
			count=0;
			cout<<"603: "<<count<<endl;
			for(int l=0;l<action_Definition_List[action_number].agents[j].size();l++)
			{
				cout<<"604: "<<count<<endl;
				for(int k=0;k<agent_name.size();k++)
				{
					cout<<"605: "<<count<<action_Definition_List[action_number].agents[j][l]<<"-"<<agent_name[k]<<endl;
					if(action_Definition_List[action_number].agents[j][l]==agent_name[k])
					{
						count++;
						cout<<"606: "<<count<<endl;
					}
				}
			}
			if(count==agent_name.size() )
			{
				if (sufficiency==true)
				{
					if(count==action_Definition_List[action_number].agents[j].size())
					{
						temp=true;
						break;
					}
				}
				else
				{
					temp=true;
					break;

				}
			}
			//					if(action_Definition_List[i].agents[j].size()>0)
			//						cout<<"this action is a joint action and a single agent can not perform it"<<endl;
		}

	}
	else if(agent_name.empty()==true && agent_type!="")// if an agent type can perform an action
	{
		cout<<610<<endl;
		for (int i=0;i<action_Definition_List.size();i++)
		{
			if(action_Definition_List[i].name==action_name)
			{
				action_number=i;
				cout<<611<<": "<<action_number<<endl;
				break;
			}
		}
		for(int j=0;j<action_Definition_List[action_number].agents.size();j++)
		{
			for(int l=0; l< action_Definition_List[action_number].agents[j].size();l++)
			{
				for(int k=0; k<agents.size(); k++)
				{
					cout<<612<<": "<<action_Definition_List[action_number].agents[j][l]<<agents[k].name<<endl;
					if(action_Definition_List[action_number].agents[j][l]==agents[k].name)
					{
						cout<<613<<": "<<action_Definition_List[action_number].agents[j][l]<<agents[k].name<<endl;
						cout<<614<<": "<<agents[k].type<<agent_type<<endl;
						if(agents[k].type==agent_type)
						{
							cout<<615<<": "<<agents[k].type<<agent_type<<endl;
							temp=true;
							break;
						}
					}
				}
				if(temp==true)
					break;
			}
			if(temp==true)
				break;
		}
	}
	else
	{
		cout<<FRED("Error, agent_name or agent_type one of them should be empty: ")<<endl;
	}

	cout<<"seq_planner_class::CanAgentPerformAction: "<<temp<<endl;
	return temp;
};

void seq_planner_class::EmergencyRobotStop(void){
	cout<<"seq_planner_class::EmergencyRobotStop"<<endl;
	emergencyFlag=true;

	for(int i=0;i<agents.size();i++)
	{
		if(agents[i].type=="Robot")
		{
			agents[i].isBusy=true;
			agents[i].lastActionAck="Stop";
			vector<string> agentName, coleaguesName; agentName.push_back(agents[i].name);

			PublishRobotAction("Stop", agentName, coleaguesName);
			agents[i].isBusy=true;
			agents[i].lastActionAck="HoldOn";
			PublishRobotAction("HoldOn", agentName, coleaguesName);

		}
	}
};

void seq_planner_class::UpdateRobotEmergencyFlag(string ActionName, vector<string>AgentsName, bool success){
	cout<<"seq_planner_class::UpdateRobotEmergencyFlag"<<endl;

	for(int i=0;i<agents.size();i++)
	{
		if(agents[i].name==AgentsName[0])
		{
			agents[i].isBusy=false;
			agents[i].isSuccessfullyDone=success;
		}
	}
	bool temp_emergencyFlag=true;
	for(int i=0;i<agents.size();i++)
	{
		if(agents[i].type=="Robot")
		{
			if(agents[i].isBusy==true || agents[i].isSuccessfullyDone==false || agents[i].lastAssignedAction!="HoldOn")
			{
				temp_emergencyFlag=false;
			}
		}
	}
	emergencyFlag=temp_emergencyFlag;

	cout<<"Updated Emergency Flag: "<<emergencyFlag<<endl;


};


void seq_planner_class::CallBackHumanAck(const std_msgs::String::ConstPtr& msg){
	cout<<"seq_planner_class::CallBackHumanAck"<<endl;

	string actionName=msg-> data.c_str();
	if(agents[0].isBusy==false)
	{
		// it means that, beforehand no action is assigned to human action
		EmergencyRobotStop();

	}

	agents[0].isBusy=false;
	agents[0].lastActionAck=actionName;
	agents[0].isSuccessfullyDone=true;
	vector<string> agentsName;
	agentsName.push_back("Human");
	bool success=true;
	//	string ActionName, vector<string>AgentsName, bool success

	if(CanAgentPerformAction(agentsName,"",actionName,false))
		UpdateStateActionTable(actionName,agentsName,success);
	else
		cout<<"Error In receiving msg from Human ack, the agents can not perform given action"<<endl;

}

void seq_planner_class::CallBackRobotAck(const std_msgs::String::ConstPtr& msg){
	cout<<"seq_planner_class::CallBackRobotAck"<<endl;

	vector<string> agentsName;
	vector<string> robot_ack_list;
	bool success;
	string delim_type="_", agents_delim_type="+";
	string robot_ack, actionName;
	bool arrived_msg=true;

	ROS_INFO("I heard Robot Ack: [%s]", msg->data.c_str());

	robot_ack=msg->data.c_str();

	boost::split(robot_ack_list, robot_ack, boost::is_any_of(delim_type));

	if(robot_ack_list.size()!=3)
	{
		cout<<"Error In receiving msg from robot ack , msg size error"<<endl;
		arrived_msg=false;
	}
	actionName=robot_ack_list[0];


	boost::split(agentsName,robot_ack_list[1], boost::is_any_of(agents_delim_type));

	if(agentsName.size()==0)
	{
		cout<<"Error In receiving msg from robot ack, agent name size is zero"<<endl;
		arrived_msg=false;
	}


	if(robot_ack_list[2]=="true")
		success=1;
	else if(robot_ack_list[2]=="false")
		success=0;
	else
	{
		cout<<"Error In receiving msg from robot"<<robot_ack_list[2]<<endl;
		arrived_msg=false;
	}

//	if
	if(CanAgentPerformAction(agentsName,"",actionName,false)==true && arrived_msg==true)
		if(emergencyFlag==false)
		{
			UpdateStateActionTable(actionName,agentsName,success);
		}
		else
		{
			UpdateRobotEmergencyFlag(actionName,agentsName,success);
		}
	else
		cout<<"Error In receiving msg from robot ack, the agents can not perform given action"<<endl;

	//	if (robot_ack=="GoalReachedLeft")
	//	{
	//		agents[1].isBusy=false;
	//		agents[1].lastActionAck=agents[1].lastAssignedAction;
	//		agents[1].isSuccessfullyDone=true;
	//		agent_update.push_back(1);
	//		UpdateStateActionTable(agent_update);
	//	}
	//	else if (robot_ack=="GoalNotReachedLeft")
	//	{
	//		agents[1].isBusy=false;
	//		agents[1].lastActionAck=agents[1].lastAssignedAction;
	//		agents[1].isSuccessfullyDone=false;
	//		agent_update.push_back(1);
	//		UpdateStateActionTable(agent_update);
	//	}
	//	else if (robot_ack=="GoalReachedRight")
	//	{
	//		agents[2].isBusy=false;
	//		agents[2].lastActionAck=agents[2].lastAssignedAction;
	//		agents[2].isSuccessfullyDone=true;
	//		agent_update.push_back(2);
	//		UpdateStateActionTable(agent_update);
	//	}
	//	else if (robot_ack=="GoalNotReachedRight")
	//	{
	//		agents[2].isBusy=false;
	//		agents[2].lastActionAck=agents[2].lastAssignedAction;
	//		agents[2].isSuccessfullyDone=false;
	//		agent_update.push_back(2);
	//		UpdateStateActionTable(agent_update);
	//	}
	//	else if (robot_ack=="GoalReachedJoint")
	//	{
	//		agents[1].isBusy=false;
	//		agents[1].lastActionAck=agents[1].lastAssignedAction;
	//		agents[1].isSuccessfullyDone=true;
	//		agents[2].isBusy=false;
	//		agents[2].lastActionAck=agents[2].lastAssignedAction;
	//		agents[2].isSuccessfullyDone=true;
	//		agent_update.push_back(1);
	//		agent_update.push_back(2);
	//		UpdateStateActionTable(agent_update);
	//	}
	//	else if (robot_ack=="GoalNotReachedBiManual")
	//	{
	//		agents[1].isBusy=false;
	//		agents[1].lastActionAck=agents[1].lastAssignedAction;
	//		agents[1].isSuccessfullyDone=false;
	//		agents[2].isBusy=false;
	//		agents[2].lastActionAck=agents[2].lastAssignedAction;
	//		agents[2].isSuccessfullyDone=false;
	//		agent_update.push_back(1);
	//		agent_update.push_back(2);
	//		UpdateStateActionTable(agent_update);
	//	}
	//	else if (robot_ack=="RobotTaskReached")
	//	{// maybe can be commented!
	//
	//		agents[1].isBusy=false;
	//		agents[1].lastActionAck=agents[1].lastAssignedAction;
	//		agents[1].isSuccessfullyDone=true;
	//		agents[2].isBusy=false;
	//		agents[2].lastActionAck=agents[2].lastAssignedAction;
	//		agents[2].isSuccessfullyDone=true;
	//		agent_update.push_back(1);
	//		agent_update.push_back(2);
	//		UpdateStateActionTable(agent_update);
	//	}
	//	else if (robot_ack=="RobotTaskNotReached")
	//	{// maybe can be commented!
	//
	//		agents[1].isBusy=false;
	//		agents[1].lastActionAck=agents[1].lastAssignedAction;
	//		agents[1].isSuccessfullyDone=false;
	//		agents[2].isBusy=false;
	//		agents[2].lastActionAck=agents[2].lastAssignedAction;
	//		agents[2].isSuccessfullyDone=false;
	//		agent_update.push_back(1);
	//		agent_update.push_back(2);
	//		UpdateStateActionTable(agent_update);
	//	}
	//	else
	//	{
	//		cout<<FRED("The arrived msg from robot is not defined here")<<endl;
	//	}
}

void seq_planner_class::PublishHumanAction(string ActionName, string AgentName, vector<string> ColleaguesName){
	cout<<"seq_planner_class::PublishHumanAction"<<endl;
	cout<<AgentName<<FBLU(": Please perform Action: ")<<ActionName<<" ";
	if(ColleaguesName.size()>0)
	{
		cout<<" , with: ";
		for(int i=0;i<ColleaguesName.size();i++){
			cout<<ColleaguesName[i]<<" ";
		}
	}
	cout<<endl;

	//	agents[0].isBusy=true;
}
void seq_planner_class::PublishRobotAction(string ActionName, vector<string> AgentsName, vector<string> ColleaguesName){
	cout<<"seq_planner_class::PublishRobotAction"<<endl;
	// publish which agent to perform an action

	string responsible_agents;
	for(int i=0;i<AgentsName.size();i++)
	{
		responsible_agents+=AgentsName[i];
		if(i<AgentsName.size()-1)
			responsible_agents+="+";
	}

	string colleagues_agents;
	for(int i=0;i<ColleaguesName.size();i++)
	{
		colleagues_agents+=ColleaguesName[i];
		if(i<ColleaguesName.size()-1)
			colleagues_agents+="+";
	}

	std_msgs::String robotMsg;
	robotMsg.data =ActionName+"_"+responsible_agents;

	if(ColleaguesName.size()>0)
		robotMsg.data +="_"+colleagues_agents;

	pubRobotCommand.publish(robotMsg);

	ROS_INFO("publish robot command : %s ",robotMsg.data.c_str() );
}
