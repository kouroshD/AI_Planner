#include "seq_planner_class.hpp"

seq_planner_class::seq_planner_class(string actionDefinitionPath,string stateActionPath){
	cout<<"seq_planner_class::seq_planner_class"<<endl;
	optimal_state=0;
	next_action_index=0;
	//	actionList=NULL;

	SetActionDefinitionList(actionDefinitionPath);
	SetAgentsList();
	SetStateActionList( stateActionPath);
	CheckStateActionList();
	cout<<"****************** Action Definition List *************************"<<endl;
	for(int i=0;i<action_Definition_List.size();i++)
		action_Definition_List[i].Print();
	cout<<"******************* Full State-Action List ************************"<<endl;
	for(int i=0;i<Full_State_action_list.size();i++)
		Full_State_action_list[i].Print();
	cout<<"********************** Agents List*********************"<<endl;
	for(int i=0;i<agents.size();i++)
		agents[i].Print();

	simulationVectorNumber=0;
	SimulationActionNumber=0;

	updateAndor=true;
	nodeSolved=false;
	haSolved=false;

	subHumanActionAck=nh.subscribe("HRecAction",100, &seq_planner_class::CallBackHumanAck, this);
	subRobotActionAck=nh.subscribe("robot_ack",100, &seq_planner_class::CallBackRobotAck, this);
	subSimulationAck=nh.subscribe("simulation_response",100, &seq_planner_class::UpdateSimulation, this);

	pubRobotCommand = nh.advertise<std_msgs::String>("robot_command",10);
	pubSimulationCommand=nh.advertise<robot_interface_msgs::SimulationRequestMsg>("simulation_command",10);

	knowledgeBase_client=nh.serviceClient<knowledge_msgs::knowledgeSRV>("knowledgeService");

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
				for (int j=0;j<state_action_table[i].actions_list.size();j++)
				{
					//					if(j==0)
					//					{
					for(int k=0;k<state_action_table[i].actions_list[j].assigned_agents.size();k++)
					{

						if(state_action_table[i].actions_list[j].isDone[k]==false)
						{// here we know where is the first action not have been done in a state:
							for(int n=0;n<AgentsName.size();n++)
							{
								cout<<"500-0: "<<state_action_table[i].actions_list[j].name<<state_action_table[i].actions_list[j].actionAndFeatures<<ActionName<<endl;
								if(state_action_table[i].actions_list[j].actionAndFeatures==ActionName)
								{
									cout<<"500-1: "<<state_action_table[i].actions_list[j].actionAndFeatures<<ActionName<<endl;
									//&&
									//state_action_table[i].actionsResponsible[j][k]==agents[agent_update].name
									for (int m=0;m<state_action_table[i].actions_list[j].assigned_agents.size();m++)
									{
										cout<<"500-2: "<<state_action_table[i].actions_list[j].assigned_agents[m]<<AgentsName[n] <<endl;

										if(state_action_table[i].actions_list[j].assigned_agents[m]==AgentsName[n])
										{
											cout<<"500-3: "<<state_action_table[i].actions_list[j].assigned_agents[m]<<AgentsName[n] <<success<<endl;

											if(success==true)
											{
												cout<<501<<" "<<temp_is_the_state_i_still_feasible<<endl;
												state_action_table[i].actions_list[j].isDone[m]=true;
												temp_is_the_state_i_still_feasible=true;
												cout<<502<<" "<<temp_is_the_state_i_still_feasible<<endl;
												break;
											}
										}

										else if(state_action_table[i].actions_list[j].assigned_agents[m]=="Unknown")
										{
											cout<<"500-4: "<<state_action_table[i].actions_list[j].assigned_agents[m]<<AgentsName[n] <<success<<endl;
											vector<string> agentNameStr;
											agentNameStr.push_back(AgentsName[n]);
											bool agent_can_perfrom_the_action=CanAgentPerformAction(agentNameStr,"",state_action_table[i].actionsList[j],false);
											// if the action an agent performed is a joint action, we should inform other agents to perfrom the action also!
											if(agent_can_perfrom_the_action==true)
											{
												if(success==true)
												{
													cout<<5010<<" "<<temp_is_the_state_i_still_feasible<<endl;
													state_action_table[i].actions_list[j].isDone[m]=true;
													state_action_table[i].actions_list[j].assigned_agents[m]=AgentsName[n];
													temp_is_the_state_i_still_feasible=true;
													cout<<5020<<" "<<temp_is_the_state_i_still_feasible<<endl;
													break;
												}

											}
											else
											{
												cout<<"The agent: "<<AgentsName[n]<<"could not perform action:"<<state_action_table[i].actions_list[j].name <<endl;
											}
										}
										else
										{
											if(state_action_table[i].actions_list[j].assigned_agents.size()==1)
											{
												cout<<"The agent: "<<AgentsName[n]<<" performed the action:"<<state_action_table[i].actions_list[j].name <<endl;
												cout<<", but the assigned agent to that is: "<< state_action_table[i].actions_list[j].assigned_agents[k]<<endl;
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
										temp_is_the_state_i_still_feasible=CanAgentPerformAction(emptyStr,"Human",state_action_table[i].actions_list[j].name, false);

									cout<<503<<" "<<temp_is_the_state_i_still_feasible<<endl;

									break;
								}

								cout<<5030011<<": "<<temp_is_the_state_i_still_feasible<<endl;

							}
							actions_done[i]=true;
							cout<<5030012<<": "<<actions_done[i]<<endl;

							for(int q=0;q<state_action_table[i].actions_list[j].isDone.size();q++)
							{
								cout<<50300070<<"****: "<<state_action_table[i].actions_list[j].name <<state_action_table[i].actions_list[j].isDone[q]	<<endl;
								if(state_action_table[i].actions_list[j].isDone[q]==false)
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
				for(int n=0;n <state_action_table[i].actions_list[state_action_table[i].actions_list.size()-1].isDone.size(); n++)
					if(state_action_table[i].actions_list[state_action_table[i].actions_list.size()-1].isDone[n]==false)
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
					FindOptimalState();
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

void seq_planner_class::FindOptimalState(void){
	cout<<"seq_planner_class::FindOptimalState"<<endl;
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

	if(number_feasible_state>0)
	{
		if(state_action_table[optimal_state].isSimulated==true)
		{
			FindNextAction();
		}
		else
		{
			GenerateOptimalStateSimulation();
		}
	}
	else
	{
		cout<<FRED("Error, There is no Feasible state now")<<endl;
		exit(1);
	}

}


void seq_planner_class::FindNextAction(){
	cout<<"seq_planner_class::FindNextAction"<<endl;
	/*!
	 * 	1- Find next action to perform
	 *
	 */


		bool next_action_progress;
		for(int i=0;i<state_action_table[optimal_state].actions_list.size();i++)
		{
			next_action_progress=false;
			for (int j=0;j<state_action_table[optimal_state].actions_list[i].isDone.size();j++)
				if (state_action_table[optimal_state].actions_list[i].isDone[j]==true)
					next_action_progress=true;

			if(next_action_progress==false)
			{
				next_action_index=i;
				break;
			}
		}
		if(next_action_progress==true){
			cout<<"In the optimal state all the actions are done!"<<endl;
			cout<<"You should check later the function, or maybe call the check_execution Function!"<<endl;
		}
		else
		{
			cout<<"Optimal State: "<<state_action_table[optimal_state].state_name<<", Next Action: "<<state_action_table[optimal_state].actionsList[next_action_index]<<endl;
			FindResponisibleAgent();
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

	if(state_action_table[optimal_state].actions_list[next_action_index].assigned_agents[0]=="Unknown")
	{
		isResponsibleAgentAcceptable=true;
		//		if(action_Definition_List[action_number].agents.size()==1)
		//		{
		//		if(action_Definition_List[action_number].agents[0].size()==1)
//		vector<bool> action_progress;
//		state_action_table[optimal_state].actionsResponsible[next_action_index].clear();
//		state_action_table[optimal_state].actionsResponsible[next_action_index]=action_Definition_List[action_number].possible_agents[0];
//		for (int i=0;i<state_action_table[optimal_state].actionsResponsible[next_action_index].size();i++)
//			action_progress.push_back(false);
//		state_action_table[optimal_state].actionsProgress[next_action_index]=action_progress;

		state_action_table[optimal_state].actions_list[next_action_index].assigned_agents.clear();
		state_action_table[optimal_state].actions_list[next_action_index].assigned_agents=state_action_table[optimal_state].actions_list[next_action_index].refActionDef.possible_agents[0];
		vector<bool> action_progress( state_action_table[optimal_state].actions_list[next_action_index].assigned_agents.size(),false);
		state_action_table[optimal_state].actions_list[next_action_index].isDone=action_progress;




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

		isResponsibleAgentAcceptable=CanAgentPerformAction(state_action_table[optimal_state].actions_list[next_action_index].assigned_agents,"",state_action_table[optimal_state].actions_list[next_action_index].name, true);
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
	//Just inside this if condition is necessary for this function, other parts should be deleted later.
	if(isResponsibleAgentAcceptable==true)
	{
		bool robotcommandSent=false;
		string action_mode;
		if(state_action_table[optimal_state].actions_list[next_action_index].assigned_agents.size()==1)
			action_mode="single";
		else
			action_mode="joint";

		vector<string> robot_agents;
		vector<string> human_colleagues;



		for(int i=0;i<state_action_table[optimal_state].actions_list[next_action_index].assigned_agents.size();i++)
		{
			for(int j=0;j<agents.size();j++)
			{
				if(state_action_table[optimal_state].actions_list[next_action_index].assigned_agents[i]==agents[j].name)
				{
					vector<string> all_colleagues;
					for(int k=0;k<state_action_table[optimal_state].actions_list[next_action_index].assigned_agents.size();k++)
						if(k!=i)
							all_colleagues.push_back(state_action_table[optimal_state].actions_list[next_action_index].assigned_agents[k]);

					agents[j].lastAssignedAction=state_action_table[optimal_state].actions_list[next_action_index].name;
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
			if(i==state_action_table[optimal_state].actions_list[next_action_index].assigned_agents.size()-1 && robot_agents.size()>0)
			{
				PublishRobotAction( state_action_table[optimal_state].actions_list[next_action_index].name, robot_agents,human_colleagues);
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

	//IMPORTANT:  I should change the place of these else condition to the state-action Definition list:
	else
	{
		cout<<FRED("The agent you defined in the 'State-Action-List' file can not perform the given action: state:")<<
				state_action_table[optimal_state].state_name<<", action:"<<state_action_table[optimal_state].actions_list[next_action_index].name<<endl;
		cout<<"Do you want to assign a new agent to it? (enter 1 if yes)";
		bool input;
		vector<string> temp_agent_list;

		cin>>input;
		if(input==true)
		{
			cout<<"Give one of the following rows as responsible:"<<endl;
			for(int m=0;m<state_action_table[optimal_state].actions_list[next_action_index].refActionDef.possible_agents.size();m++){
				for(int n=0;n<state_action_table[optimal_state].actions_list[next_action_index].refActionDef.possible_agents[m].size();n++){
					cout<<state_action_table[optimal_state].actions_list[next_action_index].refActionDef.possible_agents[m][n];
					if(n<action_Definition_List[action_number].possible_agents[m].size()-1)
						cout<<"+";
				}
				cout<<endl;
			}
			string input_string;
			cout<<"Enter the Agents: ";
			cin>>input_string;
			boost::split(temp_agent_list, input_string, boost::is_any_of("+"));
		}
		else
		{
			temp_agent_list.push_back("Unknown");
		}
		state_action_table[optimal_state].actions_list[next_action_index].assigned_agents=temp_agent_list;
		vector<bool> action_progress(temp_agent_list.size(),false);
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
		temp_obj.isSimulated=false;
		//		temp_obj.actionsList=

		for (int j=0;j<Full_State_action_list.size();j++)
		{
			if (temp_obj.state_name==Full_State_action_list[j].state_name)
			{
				nameFlag=true;
				temp_obj.actions_list=Full_State_action_list[j].actions_list;

				temp_obj.actionsList=Full_State_action_list[j].actionsList;
				temp_obj.actionsResponsible=Full_State_action_list[j].actionsResponsible;
				for (int k=0;k<temp_obj.actions_list.size();k++)
				{
					vector<bool> temp_actionProgress(temp_obj.actions_list[k].assigned_agents.size(),false);
//					for(int l=0;l<temp_obj.actions_list[k].assigned_agents.size();l++)
//						temp_actionProgress.push_back(false);
					temp_obj.actionsProgress.push_back(temp_actionProgress);
					temp_obj.actions_list[k].isDone=temp_actionProgress;
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
		if (state_action_table[i].actions_list.size()==0)
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
			int actions_size=state_action_table[i].actions_list.size();
			bool last_action_progress=true;

			for (int j=0;j<state_action_table[i].actions_list[actions_size-1].isDone.size();j++)
				if (state_action_table[i].actions_list[actions_size-1].isDone[j]==false)
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
		FindOptimalState();
	}
}

void seq_planner_class::GenerateOptimalStateSimulation(void) {
	/*!
	 *	1- Find the first action that is progressed yet, in the list:
	 *	2- Find all the parameters of the action based on state-action table (if some parameters are not assigned, we assign base on all possible parameters for that)
	 *	3- if an action could not be performed, the progress of it should be stopped for simulation.
	 *	3- if all the actions are simulated, rank them in ranking function
	 *
	 * */
	cout << "seq_planner_class::SimulateOptimalState" << endl;
	simulation_vector.clear();
	vector<optimal_state_simulation> temp_simulation_vector;
	// check for a filled parameters in actions of a state given by user-> if yes, give it to all the actions.
	optimal_state_simulation temp_sim;
	temp_sim.actions_list = state_action_table[optimal_state].actions_list;
	temp_sim.actionsTime.resize(temp_sim.actions_list.size(), 0.0);
	temp_sim.optimalStatePtr = &state_action_table[optimal_state];
	temp_sim.state_name = state_action_table[optimal_state].state_name;
	// check agents of the actions:
	// if all the agents have some assigned agents do nothing with the agents
	int agent_counter = 0;
	for (int i = 0; i < temp_sim.actions_list.size(); i++) {
		if (temp_sim.actions_list[i].assigned_agents[0] != "Unknown") {
			temp_sim.responsibleAgents =temp_sim.actions_list[i].assigned_agents;
			agent_counter++;
		}
	}
	if (agent_counter == temp_sim.actions_list.size()) {
		cout << "all the actions have assigned agents to it" << endl;
	}
	else if (agent_counter == 1)
	{
		temp_sim.SetAgentForAllTheAction();
	}
	else if (agent_counter == 0)
	{
		for (int i = 0;	i< temp_sim.actions_list[0].refActionDef.possible_agents.size();i++)
		{
			optimal_state_simulation temp_sim2 = temp_sim;
			vector<string>possibleAgents=temp_sim.actions_list[0].refActionDef.possible_agents[i];
			bool canAgentsPerformTheActions=true;
			for(int j=0;j<temp_sim.actions_list.size();j++)
			{
				if(CanAgentPerformAction(possibleAgents,"",temp_sim.actions_list[j].name,true)==false)
					canAgentsPerformTheActions=false;
			}
			if(canAgentsPerformTheActions==true)
			{
				temp_sim2.responsibleAgents =possibleAgents;
				temp_sim2.SetAgentForAllTheAction();
				temp_simulation_vector.push_back(temp_sim2);
			}
		}
		if(temp_simulation_vector.size()==0)
			cout<<"We did not found any set of agents which can perform all the actions in the optimal state"<<endl;
	}
	else
	{
		cout<< "More than one action in the state is assigned agents, please check again the state-action list"	<< endl;
		exit(1);
	}


	// check other parameters of the actions
	for (int i = 0; temp_simulation_vector[0].actions_list.size(); i++) {
		vector<string> parameter_type =
				temp_simulation_vector[0].actions_list[i].refActionDef.parameterTypes;
		bool the_parameter_is_found_before;
		for (int j = 0; j < parameter_type.size(); j++) {
			the_parameter_is_found_before = false;
			for (int k = 0;
					k < temp_simulation_vector[0].parameters_type.size();
					k++) {
				if (parameter_type[j]
						== temp_simulation_vector[0].parameters_type[k]) {
					the_parameter_is_found_before = true;
				}
			}
			if (the_parameter_is_found_before == false) {
				temp_simulation_vector[0].parameters_type.push_back(
						parameter_type[j]);
			}
		}
	}
	for (int i = 1; i < temp_simulation_vector.size(); i++) {
		temp_simulation_vector[i].parameters_type =
				temp_simulation_vector[0].parameters_type;
	}
	// the simulation vector knows how many parameter type for the actions we need, like object grasping poses, object frame, ...
	// check for first action now how many assigned parameters it can have for each type.
	for (int i = 0; i < state_action_table[optimal_state].actions_list.size(); 	i++)
	{
		for (int j = 0; j< state_action_table[optimal_state].actions_list[i].refActionDef.parameterTypes.size();j++)
		{
			string actionParameterName = state_action_table[optimal_state].actions_list[i].assignedParameters[j];
			string actionParameterType =state_action_table[optimal_state].actions_list[i].refActionDef.parameterTypes[j] + "Name";
//			int parameterNo;
//			for (int h = 0; h < temp_simulation_vector[0].parameters_type.size();h++)
//			{
//				if (temp_simulation_vector[0].parameters_type[h]== state_action_table[optimal_state].actions_list[i].refActionDef.parameterTypes[j])
//				{
//					parameterNo = h;
//					break;
//				}
//			}
			vector<string> msg1Vector;
			boost::split(msg1Vector, actionParameterName, boost::is_any_of("-"));
			knowledge_msgs::knowledgeSRV knowledge_msg;
			knowledge_msg.request.reqType = msg1Vector[0];// object, point
			knowledge_msg.request.Name = msg1Vector[1]; // 1,2,3,....
			// if msg1.size >2 ??
			knowledge_msg.request.requestInfo = actionParameterType; // graspingPose, centerPose, ...
			vector<string> responseVector;
			if (knowledgeBase_client.call(knowledge_msg))
			{
				responseVector = knowledge_msg.response.names; // here I have all the names of different grasping poses.
				// example: graspingPose1,graspingPose2
			}

			if (responseVector.size() == 0)
			{
				cout << "the knowledge base returned nothing!" << endl;
			}
			else
			{
				vector<optimal_state_simulation> temp2_simulation_vector;
				for (int m = 0; m < temp_simulation_vector.size(); m++)
				{
					int NoAgents =temp_simulation_vector[m].responsibleAgents.size();
					vector<string> AssignedParametersCombinations;
					PossibileCombinations(responseVector, NoAgents,	AssignedParametersCombinations);
					for (int n = 0; n < AssignedParametersCombinations.size(); 	n++)
					{
						temp_simulation_vector[m].actions_list[i].assignedParameters[j]= actionParameterName+"-"+AssignedParametersCombinations[n];
						temp2_simulation_vector.push_back(temp_simulation_vector[m]);
					}
				}
				temp_simulation_vector.clear();
				temp_simulation_vector=temp2_simulation_vector;
			}
		}
	}
	simulation_vector=temp_simulation_vector;

	if(simulation_vector.size()==0)
	{
		cout<<" Error: The Simulation Vector is Empty"<<endl;
	}
	else
	{
		GiveSimulationCommand();
	}


	// give simulation command to the robot interface:

}

void seq_planner_class::GiveSimulationCommand(void){
	// 1- find the first command it should publish
	// 2- Fill the msg for giving the command
	// 3- publish the command

	bool breakFlag=false;

	for(int i=0;i< simulation_vector[0].actions_list.size();i++)
	{
		for(int j=0;j< simulation_vector.size();i++)
		{
			if(simulation_vector[j].actions_list[i].isDone[0]==false)
			{
				simulationVectorNumber=j;
				SimulationActionNumber=i;
				breakFlag=true;
				break;
			}
		}
		if(breakFlag==true)
			break;
	}
	robot_interface_msgs::SimulationRequestMsg req_instance;
	robot_interface_msgs::Joints joint_values;

	for(int i=0; i<2;i++)
		for(int j=0;j<7;j++)
		{
			joint_values.values[j]=simulation_vector[simulationVectorNumber].simulation_q[i][j];
			req_instance.ArmsJoint.push_back(joint_values);
		}

	string simulationCommandStr;

	// add actions name to the msg
	req_instance.ActionName=simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].name;


	// add actions parameter names +info to the msg
	for (int i=0;i<simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].assignedParameters.size();i++)
	{
		req_instance.ActionParametersName.push_back(simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].assignedParameters[i]);
		req_instance.ActionParameterInfo.push_back( simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].refActionDef.parameterTypes[i]);
	}
//	// add responsible agents to the msg
	simulationCommandStr=simulationCommandStr+" ";
	vector<string> temp_colleagues;
	for (int i=0;i<simulation_vector[simulationVectorNumber].responsibleAgents.size();i++)
	{
		if( ResponsibleAgentType(simulation_vector[simulationVectorNumber].responsibleAgents[i])=="Human" )
			req_instance.ColleagueAgents.push_back(simulation_vector[simulationVectorNumber].responsibleAgents[i]);
		else
			req_instance.ResponsibleAgents.push_back(simulation_vector[simulationVectorNumber].responsibleAgents[i]);
	}

	pubSimulationCommand.publish(req_instance);

}

void seq_planner_class::UpdateSimulation(const robot_interface_msgs::SimulationResponseMsg&  simulationResponse){

	// 1- Fill the necessary data inside the simulation vector
	// 2- if an action is done completely, fill also all the consequential action from other simulation vector
	// 3- if all the actions in all the simulation vectors is done, go to the RankSimulation Functions, otherwise go to the GiveCommand Function

	bool failure_Flag=false;
	for(int i=0;i< simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].assigned_agents.size();i++)
	{
		for(int j=0; j< simulationResponse.ResponsibleAgents.size();j++)
		{
			if(simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].assigned_agents[i]==
					simulationResponse.ResponsibleAgents[j])
			{
				if(simulationResponse.success==true)
				{

					simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].isDone[i]=true;
					// fill the simulation timing:
					if(simulationResponse.time>simulation_vector[simulationVectorNumber].actionsTime[SimulationActionNumber])
						simulation_vector[simulationVectorNumber].actionsTime[SimulationActionNumber]=simulationResponse.time;

					// Fill the simulation joint values:
					if(simulationResponse.ResponsibleAgents[j]=="LeftArm")
					{
						for(int k=0;k<7;k++)
						simulation_vector[simulationVectorNumber].simulation_q[0][k]=simulationResponse.ArmsJoint[0].values[k];
					}
					else if(simulationResponse.ResponsibleAgents[j]=="RightArm")
					{
						for(int k=0;k<7;k++)
						simulation_vector[simulationVectorNumber].simulation_q[1][k]=simulationResponse.ArmsJoint[1].values[k];
					}
					else{}
					failure_Flag=false;

				}
				else
				{
					// break all the for loops
					// the action can not be performed, Delete that simulation and all the other similar ones from simulation vector
					// give a new simulation command to the robot

					failure_Flag=true;
				}
				break;
			}
		}
		if(failure_Flag==true)
			break;
	}

	// if the simulation shows failure:
	// the action can not be performed, Delete that simulation and all the other similar ones from simulation vector
	// give a new simulation command to the robot
	if(failure_Flag==true)
	{

		for(vector<optimal_state_simulation>::iterator it =simulation_vector.begin(); it!= simulation_vector.end();)
		{
			if(isTwoActionsEqual(simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber],(*it).actions_list[SimulationActionNumber] ))
			{
				simulation_vector.erase(it);
			}
			else
				it++;
		}
		//GiveSimulationCommand(); // should be given later not here

	}
	else // the simulation shows successful implementation:
	{
		bool actionCompletelydone=true;
		for(int i=0; i<simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].isDone.size();i++)
			if(simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].isDone[i]==false)
				actionCompletelydone=false;

		if(actionCompletelydone==true)
		{
			for(vector<optimal_state_simulation>::iterator it =simulation_vector.begin(); it!= simulation_vector.end();it++)
				if(isTwoActionsEqual(simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber],(*it).actions_list[SimulationActionNumber] ))
					for(int j=0;j< (*it).actions_list[SimulationActionNumber].isDone.size();j++)
					{
						(*it).actions_list[SimulationActionNumber].isDone[j]=true; // we are sure all of them is true
						(*it).actionsTime[SimulationActionNumber]=simulation_vector[simulationVectorNumber].actionsTime[SimulationActionNumber];

						if((*it).actions_list[SimulationActionNumber].assigned_agents[j]=="LeftArm")
						{
							for(int k=0;k<7;k++)
								(*it).simulation_q[0][k]=simulation_vector[simulationVectorNumber].simulation_q[0][k];
						}
						else if((*it).actions_list[SimulationActionNumber].assigned_agents[j]=="RightArm")
						{
							for(int k=0;k<7;k++)
								(*it).simulation_q[1][k]=simulation_vector[simulationVectorNumber].simulation_q[1][k];
						}
						else{}
					}
		}
	}

	bool OptimalStateCompletelySimulated=true;
	int lastIndex=simulation_vector[0].actions_list.size()-1;
	for(int i=0;i<simulation_vector.size();i++)
	{
		for(int j=0;j<simulation_vector[i].actions_list[lastIndex].isDone.size();j++)
			if(simulation_vector[i].actions_list[lastIndex].isDone[j]==false)
			{
				OptimalStateCompletelySimulated=false;
				break;
			}
		if(OptimalStateCompletelySimulated==false)
			break;
	}
	if(OptimalStateCompletelySimulated==true)
		RankSimulation();
	else
		GiveSimulationCommand();
}

void seq_planner_class::RankSimulation(void){



}



string seq_planner_class::ResponsibleAgentType(string agent_name){
	string temp_agent_type;

	for (int i=0;i<agents.size();i++)
	{
		if(agents[i].name==agent_name)
		{
			temp_agent_type=agents[i].type;
			break;
		}
	}

	return temp_agent_type;
}

void seq_planner_class::SetActionDefinitionList(string actionDefinitionPath){

	cout<<"seq_planner_class::SetActionDefinitionList"<<endl;
	//	ROS_INFO("%s",actionDefinitionPath.c_str());
	ifstream file_path_ifStr(actionDefinitionPath.c_str());
	string delim_type=" ";
	string delim_agent="/";
	string delim_joint="+";
	string delim_actionParameters="_";

	std::vector<std::string> line_list;
	string line;

	cout<<file_path_ifStr.is_open()<<endl;
	if (file_path_ifStr.is_open())
	{
		while(getline(file_path_ifStr,line))
		{
			actionDef actionDef;
			boost::split(line_list, line, boost::is_any_of(delim_type));
			if(line_list[0]!="#")
			{
				vector<string> action_paramters;
				boost::split(action_paramters, line_list[0], boost::is_any_of(delim_actionParameters));
				actionDef.name=action_paramters[0];
				if(action_paramters.size()>1)
					for(int i=1;i<action_paramters.size();i++)
						actionDef.parameterTypes.push_back(action_paramters[i]);

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
					actionDef.possible_agents.push_back(joint_agents);
					counter++;
				}
				action_Definition_List.push_back(actionDef);
			}
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
	vector<string> action_and_responsibles,action_and_parameters;

	string line;
	string delim_type=" ";
	string responsible_delim_type="->";
	string jointAction_delim_type="+";
	string actionParameter_delim_type="_";


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

				actionDef tempActionDef;
				boost::split(action_and_responsibles, line_list[i], boost::is_any_of(responsible_delim_type));
				boost::split(action_and_parameters, action_and_responsibles[0], boost::is_any_of(actionParameter_delim_type));
				bool actionName=false;
				for(int j=0;j<action_Definition_List.size();j++)
					if(action_and_parameters[0]==action_Definition_List[j].name)
					{
						actionName=true;
						tempActionDef=action_Definition_List[j];
						break;
					}
				if(actionName==false){
					cout<<"The written action is not defined in actions definition list, please check: state: "<<line_list[0]<<", action: "<<action_and_parameters[0]<<endl;
					exit(1);
				}
				action tempAction(tempActionDef);
				tempAction.actionAndFeatures=action_and_responsibles[0];


				for(int j=1;j<action_and_parameters.size();j++){
					tempAction.assignedParameters.push_back(action_and_parameters[j]);
				}


				temp_obj.actionsList.push_back(action_and_responsibles[0]);
				if(action_and_responsibles.size()==2)
				{
					boost::split(responsibles, action_and_responsibles[1], boost::is_any_of(jointAction_delim_type));
					temp_obj.actionsResponsible.push_back(responsibles);
					tempAction.assigned_agents=responsibles;
				}
				else if(action_and_responsibles.size()==1)
				{
					responsibles.push_back("Unknown");
					temp_obj.actionsResponsible.push_back(responsibles);
					tempAction.assigned_agents=responsibles;
				}
				else
				{cout<<"Error in state_action text file, please check state: "<<temp_obj.state_name<<", action: "<<action_and_responsibles[0]<<endl;}
				temp_obj.actions_list.push_back(tempAction);
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

void seq_planner_class::CheckStateActionList(){
	cout<<"seq_planner_class::CheckStateActionList"<<endl;
	bool isResponsibleAgentAcceptable;

	for(int i=0;i<Full_State_action_list.size();i++)
	{
		for(int j=0;j<Full_State_action_list[i].actions_list.size();j++)
		{
			isResponsibleAgentAcceptable=false;
			if(Full_State_action_list[i].actions_list[j].assigned_agents[0]=="Unknown")
			{
				isResponsibleAgentAcceptable=true;
				// if the action have one set of agents to perfrom assign it here:
				if(Full_State_action_list[i].actions_list[j].refActionDef.possible_agents.size()==1)
				{
					Full_State_action_list[i].actions_list[j].assigned_agents=Full_State_action_list[i].actions_list[j].refActionDef.possible_agents[0];
				}
			}
			else
			{
				isResponsibleAgentAcceptable=CanAgentPerformAction(Full_State_action_list[i].actions_list[j].assigned_agents,"",Full_State_action_list[i].actions_list[j].name, true);
			}

			if(isResponsibleAgentAcceptable==false)
			{
				cout<<FRED("The agent you defined in the 'Full_State_action_list' file can not perform the given action: state:")<<
						Full_State_action_list[i].state_name<<", action:"<<Full_State_action_list[i].actions_list[j].name<<endl;
				cout<<"Do you want to assign a new agent to it? (enter 1 if yes)";
				bool input;
				vector<string> temp_agent_list;

				cin>>input;
				if(input==true)
				{
					cout<<"Give one of the following rows as responsible:"<<endl;
					for(int m=0;m<Full_State_action_list[i].actions_list[j].refActionDef.possible_agents.size();m++)
					{
						for(int n=0;n<Full_State_action_list[i].actions_list[j].refActionDef.possible_agents[m].size();n++)
						{
							cout<<Full_State_action_list[i].actions_list[j].refActionDef.possible_agents[m][n];
							if(n<Full_State_action_list[i].actions_list[j].refActionDef.possible_agents[m].size()-1)
								cout<<"+";
						}
						cout<<endl;
					}
					string input_string;
					cout<<"Enter the Agents: ";
					cin>>input_string;
					boost::split(temp_agent_list, input_string, boost::is_any_of("+"));
				}
				else
				{
					temp_agent_list.push_back("Unknown");
				}
				Full_State_action_list[i].actions_list[j].assigned_agents=temp_agent_list;
			}

		}
	}

};



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
		for(int j=0;j<action_Definition_List[action_number].possible_agents.size();j++)
		{
			count=0;
			cout<<"603: "<<count<<endl;
			for(int l=0;l<action_Definition_List[action_number].possible_agents[j].size();l++)
			{
				cout<<"604: "<<count<<endl;
				for(int k=0;k<agent_name.size();k++)
				{
					cout<<"605: "<<count<<action_Definition_List[action_number].possible_agents[j][l]<<"-"<<agent_name[k]<<endl;
					if(action_Definition_List[action_number].possible_agents[j][l]==agent_name[k])
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
					if(count==action_Definition_List[action_number].possible_agents[j].size())
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
		for(int j=0;j<action_Definition_List[action_number].possible_agents.size();j++)
		{
			for(int l=0; l< action_Definition_List[action_number].possible_agents[j].size();l++)
			{
				for(int k=0; k<agents.size(); k++)
				{
					cout<<612<<": "<<action_Definition_List[action_number].possible_agents[j][l]<<agents[k].name<<endl;
					if(action_Definition_List[action_number].possible_agents[j][l]==agents[k].name)
					{
						cout<<613<<": "<<action_Definition_List[action_number].possible_agents[j][l]<<agents[k].name<<endl;
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
	vector<string> agentName, coleaguesName;


	for(int i=0;i<agents.size();i++)
	{
		agentName.clear();
		if(agents[i].type=="Robot")
		{
			agents[i].isBusy=true;
			agents[i].isSuccessfullyDone=false;

			agentName.push_back(agents[i].name);

			if(agents[i].lastAssignedAction=="Stop")
			{
				agents[i].lastAssignedAction="HoldOn";
				PublishRobotAction("HoldOn", agentName, coleaguesName);
			}
			else
			{
				agents[i].lastAssignedAction="Stop";
				PublishRobotAction("Stop", agentName, coleaguesName);
			}
		}

	}
};

void seq_planner_class::UpdateRobotEmergencyFlag(string ActionName, vector<string>AgentsName, bool success){
	cout<<"seq_planner_class::UpdateRobotEmergencyFlag"<<endl;

	for(int i=0;i<agents.size();i++)
	{
		if(agents[i].name==AgentsName[0])
		{
			cout<<"800"<<endl;
			agents[i].isBusy=false;
			agents[i].isSuccessfullyDone=success;
			agents[i].lastActionAck=ActionName;
			agents[i].Print();
		}
	}

	vector<bool> emergencyFlagVector, holdOnFlag;
	cout<<"*****************800-1**************"<<endl;
	int count=0;
	for(int i=0;i<agents.size();i++)
	{
		if(agents[i].type=="Robot")
		{
			agents[i].Print();
			emergencyFlagVector.push_back(false);
			holdOnFlag.push_back(false);

			if(agents[i].isBusy==false && agents[i].isSuccessfullyDone==true &&  agents[i].lastActionAck=="Stop" &&  agents[i].lastAssignedAction=="Stop" )
			{
				holdOnFlag[count]=true;
				cout<<"803"<<endl;
			}
			else if(agents[i].isBusy==false && agents[i].isSuccessfullyDone==true &&  agents[i].lastActionAck=="HoldOn" && agents[i].lastAssignedAction=="HoldOn")
			{
				emergencyFlagVector[count]=true;
				cout<<"804"<<endl;
			}
			else
			{}
			count++;
		}
	}
	bool temp_EmergencyFlag=true, temp_HoldOnFlag=true;
	count=0;
	cout<<"*****************800-2**************"<<endl;
	for(int i=0;i<agents.size();i++)
	{
		if(agents[i].type=="Robot")
		{
			agents[i].Print();
			if(holdOnFlag[count]==false)
			{
				cout<<"*****"<<805<<endl;
				temp_HoldOnFlag=false;


			}
			 if(emergencyFlagVector[count]==false)
			{
				cout<<"*****"<<806<<endl;
				temp_EmergencyFlag=false;
			}
			count++;
		}
	}

	cout<<"Updated hold on Flag: "<<temp_HoldOnFlag<<endl;
	cout<<"Updated Emergency Flag: "<<temp_EmergencyFlag<<endl;

	if(temp_HoldOnFlag==true)
		EmergencyRobotStop();
	else if(temp_EmergencyFlag==true)
		emergencyFlag=!temp_EmergencyFlag;
	else
	{}

	cout<<"emergency Flag: "<<emergencyFlag<<endl;
};


void seq_planner_class::CallBackHumanAck(const std_msgs::String::ConstPtr& msg){
	cout<<"seq_planner_class::CallBackHumanAck"<<endl;

	string actionName=msg-> data.c_str();
	cout<<agents[0].name<<" action: "<< actionName<<endl;
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

	cout<<"emergency Flag: "<<emergencyFlag<<endl;
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
	robotMsg.data =ActionName+" "+responsible_agents;

	if(ColleaguesName.size()>0)
		robotMsg.data +=" "+colleagues_agents;

	pubRobotCommand.publish(robotMsg);

	ROS_INFO("publish robot command : %s ",robotMsg.data.c_str() );
}
