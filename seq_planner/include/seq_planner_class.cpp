#include "seq_planner_class.hpp"

seq_planner_class::seq_planner_class(string seqPlannerPath,string AssemblyName){
	cout<<BOLD(FBLU("seq_planner_class::seq_planner_class"))<<endl;
	const char* DataLogPath	="/home/nasa/Datalog/ROMAN2018/0";
	string DataLogPathStr (DataLogPath);
	mkdir(DataLogPath , S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	fileLog.open((DataLogPathStr+"/1_Assembly_Timing.txt").c_str(),ios::app);
//	simFileLog.open((DataLogPathStr+"/1_SimulationFileLog.txt").c_str(),ios::app);
	timeNow=ros::Time::now().toSec();

	fileLog<< to_string(timeNow)<<" cooperation started"<<"\n";
	fileLog<< to_string(timeNow)<<" offline started"<<"\n";

	optimal_state=0;
	next_action_index=0;
	//	actionList=NULL;
	seq_planner_path=seqPlannerPath;
	assembly_name=AssemblyName;
	AndOrUpdateName=assembly_name;
	hierarchicalGraphList.push_back(assembly_name);

	ReadObjectsType(seq_planner_path+"/objectTypes.txt");
	SetActionDefinitionList(seq_planner_path+"/ActionDefinitionList_"+assembly_name+".txt");
	SetAgentsList();
	Full_State_action_list.resize(1);
	Full_State_action_list[0].graph_name=assembly_name;
	SetStateActionList( seq_planner_path+"/StateActionList_"+assembly_name+".txt",assembly_name, Full_State_action_list[0].graph_state_action_offline_vector);
	if(complexActionsList.size()>0)
	{
		Full_State_action_list.resize(1+complexActionsList.size());
		for(int i=0;i<complexActionsList.size();i++)
		{
			Full_State_action_list[i+1].graph_name=complexActionsList[i];
			SetStateActionList( seq_planner_path+"/StateActionList_"+complexActionsList[i]+".txt",complexActionsList[i] ,Full_State_action_list[i+1].graph_state_action_offline_vector);
		}
	}
	CheckStateActionList();
	cout<<FBLU(BOLD("*******************************************************************"))<<endl;
	cout<<FBLU(BOLD("****************** Action Definition List *************************"))<<endl;
	for(int i=0;i<action_Definition_List.size();i++)
		action_Definition_List[i].Print();
	cout<<FBLU(BOLD("*******************************************************************"))<<endl;
	cout<<FBLU(BOLD("******************* Full State-Action List ************************"))<<endl;
	for(int i=0;i<Full_State_action_list.size();i++)
		Full_State_action_list[i].Print();
	cout<<FBLU(BOLD("*******************************************************************"))<<endl;
	cout<<FBLU(BOLD("*************************** Agents List ***************************"))<<endl;
	for(int i=0;i<agents.size();i++)
		agents[i].Print();

	cout<<FBLU(BOLD("*******************************************************************"))<<endl;
	cout<<FBLU(BOLD("*************************** Object Type List ***************************"))<<endl;
	for(int i=0;i<objectTypeVector.size();i++)
		cout<<objectTypeVector[i]<<" ";
	cout<<endl;


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
	pubToRobotDisplay=nh.advertise<std_msgs::String>("robotDisplayText",10);

	emergencyFlag=false;

	timeNow=ros::Time::now().toSec();
	fileLog<< to_string(timeNow)<<" offline ended"<<"\n";
	fileLog<< to_string(timeNow)<<" online started"<<"\n";
	fileLog<< to_string(timeNow)<<" andor started"<<"\n";


}
seq_planner_class::~seq_planner_class(){
	cout<<"seq_planner_class::~seq_planner_class"<<endl;
	fileLog.close();
//	simFileLog.close();

}

void seq_planner_class::UpdateStateActionTable_ComplexAction(string ActionName, bool success){
	// The action name and the parameters of the action should be equal for two actions to be equal

	if(state_action_table.size()>0)
	{
		for(int i=0;i<state_action_table.size();i++)
		{
			bool temp_is_the_state_i_still_feasible=false;
			if (state_action_table[i].isFeasible==true)
			{
				bool break_flag=false;
				for (int j=0;j<state_action_table[i].actions_list.size();j++)
				{
					if(state_action_table[i].actions_list[j].isDone[0]==false)
					{
						if(state_action_table[i].actions_list[j].Action_GeneralParameters==ActionName && state_action_table[i].actions_list[j].refActionDef.actionType=="complex")
						{
							if(success==true)
							{
								state_action_table[i].actions_list[j].isDone[0]=true;
								temp_is_the_state_i_still_feasible=true;
							}
							// else: the state will be infeasible

						}
						else
						{
							vector<string> emptyStr;
							temp_is_the_state_i_still_feasible=CanAgentPerformAction(emptyStr,"Human",state_action_table[i].actions_list[j].name, false);

//							cout<<503<<" "<<temp_is_the_state_i_still_feasible<<endl;

							break;
						}

						break;
					}

				}
			}
		}
	}

};

void seq_planner_class::UpdateStateActionTable(string ActionName, vector<string>AgentsName, bool success){
	cout<<BOLD(FBLU("seq_planner_class::UpdateStateActionTable"))<<endl;
	/*! when an acknowledgment arrive from agents:
		 1- Search for action in state-action table
		 2- Update the following state (take the feasible state with minimum cost)
		 	 - if an state is executed inform the andor graph, come out of here
	 */

	//	cout<<FBLU("before update: ")<<endl;
	//	for (int i=0;i<state_action_table.size();i++)
	//		state_action_table[i].Print();

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
						{
							// here we know where is the first action not have been done in a state:
							for(int n=0;n<AgentsName.size();n++)
							{
//								cout<<"500-0: "<<state_action_table[i].actions_list[j].name<<state_action_table[i].actions_list[j].actionAndParameters<<ActionName<<endl;
								if(state_action_table[i].actions_list[j].actionAndParameters==ActionName)
								{
//									cout<<"500-1: "<<state_action_table[i].actions_list[j].actionAndParameters<<ActionName<<endl;
									//&&
									//state_action_table[i].actionsResponsible[j][k]==agents[agent_update].name
									for (int m=0;m<state_action_table[i].actions_list[j].assigned_agents.size();m++)
									{
//										cout<<"500-2: "<<state_action_table[i].actions_list[j].assigned_agents[m]<<AgentsName[n] <<endl;

										if(state_action_table[i].actions_list[j].assigned_agents[m]==AgentsName[n])
										{
//											cout<<"500-3: "<<state_action_table[i].actions_list[j].assigned_agents[m]<<AgentsName[n] <<success<<endl;

											if(success==true)
											{
//												cout<<501<<" "<<temp_is_the_state_i_still_feasible<<endl;
												state_action_table[i].actions_list[j].isDone[m]=true;
												temp_is_the_state_i_still_feasible=true;
//												cout<<502<<" "<<temp_is_the_state_i_still_feasible<<endl;
												break;
											}
										}

										else if(state_action_table[i].actions_list[j].assigned_agents[m]=="Unknown")
										{
//											cout<<"500-4: "<<state_action_table[i].actions_list[j].assigned_agents[m]<<AgentsName[n] <<success<<endl;
											vector<string> agentNameStr;
											agentNameStr.push_back(AgentsName[n]);
											bool agent_can_perfrom_the_action=CanAgentPerformAction(agentNameStr,"",state_action_table[i].actions_list[j].name,false);
											// if the action an agent performed is a joint action, we should inform other agents to perfrom the action also!
											if(agent_can_perfrom_the_action==true)
											{
												if(success==true)
												{
//													cout<<5010<<" "<<temp_is_the_state_i_still_feasible<<endl;
													state_action_table[i].actions_list[j].isDone[m]=true;
													state_action_table[i].actions_list[j].assigned_agents[m]=AgentsName[n];
													temp_is_the_state_i_still_feasible=true;
//													cout<<5020<<" "<<temp_is_the_state_i_still_feasible<<endl;
													break;
												}

											}
											else
											{
//												cout<<"The agent: "<<AgentsName[n]<<"could not perform action:"<<state_action_table[i].actions_list[j].name <<endl;
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

//									cout<<503<<" "<<temp_is_the_state_i_still_feasible<<endl;

									break;
								}

//								cout<<5030011<<": "<<temp_is_the_state_i_still_feasible<<endl;

							}
							actions_done[i]=true;
//							cout<<5030012<<": "<<actions_done[i]<<endl;

							for(int q=0;q<state_action_table[i].actions_list[j].isDone.size();q++)
							{
//								cout<<50300070<<"****: "<<state_action_table[i].actions_list[j].name <<state_action_table[i].actions_list[j].isDone[q]	<<endl;
								if(state_action_table[i].actions_list[j].isDone[q]==false && temp_is_the_state_i_still_feasible==false)
								{
									actions_done[i]=false;
								}

							}
//							cout<<5030013<<": "<<actions_done[i]<<endl;

//							cout<<5030014<<": "<<temp_is_the_state_i_still_feasible<<endl;
							break_flag=true;
							break; //2 breaks i need
						}
//						cout<<5030015<<": "<<temp_is_the_state_i_still_feasible<<endl;
					}

					if(break_flag==true){
//						cout<<503002<<": "<<temp_is_the_state_i_still_feasible<<endl;
						break;
					}
//					cout<<503003<<": "<<temp_is_the_state_i_still_feasible<<endl;

				}
//				cout<<503004<<": "<<temp_is_the_state_i_still_feasible<< state_action_table[i].isFeasible<< " "<< state_action_table[i].state_name<<endl;
				if(temp_is_the_state_i_still_feasible==true)
				{
//					cout<<504<<" "<<temp_is_the_state_i_still_feasible<<endl;
					state_action_table[i].isFeasible=true;
					is_a_state_feasible=true;
				}
				else
				{
//					cout<<505<<" "<<temp_is_the_state_i_still_feasible<<endl;
					state_action_table[i].isFeasible=false;
				}

			}

			if(state_action_table[i].isFeasible==true)
			{
//				cout<<506<<" "<<state_action_table[i].isFeasible<<endl;
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


//		cout<<"550: "<<is_a_state_solved<<endl;

		for(int h=0;h<actions_done.size();h++)
			if(actions_done[h]==true)
				is_an_action_done=true;

		if(is_a_state_solved==false)
		{
//			cout<<"551: "<<is_a_state_solved<<is_a_state_feasible<<is_an_action_done<<endl;
			if(is_a_state_feasible==true)
			{

				//				cout<<FBLU("after update: ")<<endl;
				//				for (int i=0;i<state_action_table.size();i++)
				//					state_action_table[i].Print();
				if(is_an_action_done==true)
				{
					FindOptimalState();
				}
			}
			else
			{
				timeNow=ros::Time::now().toSec();
				fileLog<< to_string(timeNow)<<" planning ended"<<"\n";
				fileLog<< to_string(timeNow)<<" online ended"<<"\n";
				fileLog<< to_string(timeNow)<<" cooperation ended"<<"\n";
				fileLog.close();

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
	cout<<BOLD(FBLU("seq_planner_class::FindOptimalState"))<<endl;
	/*!
	 * Find the optimal state
	 * If the optimal state has a complex action:
	 * 		if there is no graph with the complex action name:
	 * 			Call the and/or graph and update the state-action table
	 * 		if there is a graph name in state-action table with the name of the complex action:
	 *			If there is a feasible state in the set of states of the complex action:
	 *				update the optimal state index with this minimum cost feasible state
	 *			else:
	 *				make the state which there is the complex action as infeasible
	 *				find new optimal state
	 */

	int state_min_cost=10000; // a high value
	int number_feasible_state=0;
	if(hierarchicalGraphList.size()==0)
	{
		cout<<"The hierarchical graph list is zero!"<<endl;
		exit(1);
	}
	for(int i=0;i<state_action_table.size();i++)
	{
		if(state_action_table[i].isFeasible==true && state_action_table[i].andorName==hierarchicalGraphList.back())
		{
			number_feasible_state++;
			if (state_action_table[i].state_cost<state_min_cost)
			{
				state_min_cost=state_action_table[i].state_cost;
				optimal_state=i;
			}
		}
	}

	// Check if the optimal state, there is a complex action, add call the new and/or graph, and add it the feasible states/actions to the state-action list
	for(int i=0;i<state_action_table[optimal_state].actions_list.size();i++)
	{
		if(state_action_table[optimal_state].actions_list[i].refActionDef.actionType=="complex")
		{
//			cout<<" 801 -------------- Call Complex action, andor"<<endl;
			updateAndor=true;
			AndOrUpdateName=state_action_table[optimal_state].actions_list[i].refActionDef.name;
			hierarchicalGraphList.push_back(AndOrUpdateName);

			timeNow=ros::Time::now().toSec();
			fileLog<< to_string(timeNow)<<" planning ended"<<"\n";
			fileLog<< to_string(timeNow)<<" andor started"<<"\n";

			return;
		}
	}

	if(number_feasible_state>0)
	{
		cout<<"optimal state: "<<state_action_table[optimal_state].state_name<<endl;
		if(state_action_table[optimal_state].isSimulated==true)
		{
			return FindNextAction();
		}
		else
		{

			return GenerateOptimalStateSimulation();
		}
	}
	else
	{
		timeNow=ros::Time::now().toSec();
		fileLog<< to_string(timeNow)<<" planning ended"<<"\n";
		fileLog<< to_string(timeNow)<<" online ended"<<"\n";
		fileLog<< to_string(timeNow)<<" cooperation ended"<<"\n";
		fileLog.close();

		cout<<FRED("Error, There is no Feasible state now")<<endl;
		exit(1);
	}

}


void seq_planner_class::FindNextAction(){
	cout<<BOLD(FBLU("seq_planner_class::FindNextAction"))<<endl;
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
	if(next_action_progress==true)
	{
		cout<<"In the optimal state all the actions are done!"<<endl;
		cout<<"You should check later the function, or maybe call the check_execution Function!"<<endl;
		CheckStateExecution();
	}
	else
	{
		cout<<"Optimal State: "<<state_action_table[optimal_state].state_name<<", Next Action: "<<state_action_table[optimal_state].actions_list[next_action_index].name<<endl;
		FindResponisibleAgent();
	}
}
void seq_planner_class::FindResponisibleAgent(void){
	cout<<BOLD(FBLU("seq_planner_class::FindResponisibleAgent"))<<endl;

	if(state_action_table[optimal_state].actions_list[next_action_index].assigned_agents[0]=="Unknown")
		cout<<BOLD(FRED("The action does not have an assigned agent to it"))<<endl;


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

				agents[j].lastAssignedAction=state_action_table[optimal_state].actions_list[next_action_index].actionAndParameters;
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
			PublishRobotAction( state_action_table[optimal_state].actions_list[next_action_index].actionAndParameters, robot_agents,human_colleagues);
		}
	}

}

void seq_planner_class::GenerateStateActionTable(vector<vector<string>> gen_Feasible_state_list, vector<int> gen_Feasible_stateCost_list, string graphName, bool graphSolved){
	cout<<BOLD(FBLU("seq_planner_class::GenerateStateActionTable"))<<endl;
	timeNow=ros::Time::now().toSec();
	fileLog<< to_string(timeNow)<<" andor ended"<<"\n";
	fileLog<< to_string(timeNow)<<" planning started"<<"\n";


	// delete the states from state-action list which the graph name is equal to the updated graph
	int aa;
//	cin>>aa;
//	cout<<FBLU("before delete: ")<<endl;
//	for (int i=0;i<state_action_table.size();i++)
//		state_action_table[i].PrintSummary();
	//	cin>>aa;

	vector<feasible_state_action> temp_state_action_table=state_action_table;
	state_action_table.clear();

	for(int i=0;i<temp_state_action_table.size();i++)
		if(temp_state_action_table[i].andorName!=graphName)
			state_action_table.push_back(temp_state_action_table[i]);

	//	for(vector<feasible_state_action>::iterator it =state_action_table.begin(); it!= state_action_table.end();)
	//	{
	//		if((*it).andorName==graphName)
	//		{
	//			state_action_table.erase(it);
	//		}
	//		else
	//		{it++;}
	//	}

//	cout<<FBLU("after delete: ")<<endl;
//	for (int i=0;i<state_action_table.size();i++)
//		state_action_table[i].PrintSummary()();

	//	cin>>aa;

	//	cout<<FBLU(BOLD("*******************************************************************"))<<endl;
	//	cout<<FBLU(BOLD("******************* Full State-Action List ************************"))<<endl;
	//	for(int i=0;i<Full_State_action_list.size();i++)
	//		for(int j=0;j<Full_State_action_list[i].size();j++)
	//		Full_State_action_list[i][j].Print();
	//	cin>>aa;


	// surely the graph name here is not equal to the assembly graph name, it the assembly graph name is solved, the program will exit before in the main
	if(graphSolved==true) // it is an complex action
	{
		//The cooperation is done
		if(graphName==assembly_name)
		{
			cout<<BOLD(FGRN("The Assembly task is Done"))<<endl;

			timeNow=ros::Time::now().toSec();
			fileLog<< to_string(timeNow)<<" online ended"<<"\n";
			fileLog<< to_string(timeNow)<<" cooperation ended"<<"\n";
			fileLog.close();
			std_msgs::String msgToDisplay;
			msgToDisplay.data="Cooperation_Done Human";
			pubToRobotDisplay.publish(msgToDisplay);

			usleep(0.5e6);
			exit(1);
		}
		else
		{
		hierarchicalGraphList.pop_back();
		vector<string>AgentsName;AgentsName.push_back("Human");
		return UpdateStateActionTable(graphName, AgentsName,graphSolved);
		}
	}

	//	cout<<"100: "<<gen_Feasible_stateCost_list.size()<<endl;

	for(int i=0; i<gen_Feasible_stateCost_list.size();i++)
	{
		bool nameFlag=false;
		feasible_state_action temp_obj;
		temp_obj.state_name=gen_Feasible_state_list[i][0];
		temp_obj.state_type=gen_Feasible_state_list[i][1];
		temp_obj.state_cost=gen_Feasible_stateCost_list[i];
		temp_obj.isFeasible=true;
		temp_obj.isSimulated=false;
		temp_obj.andorName=graphName;

//		cout<<"Full_State_action_list: "<<Full_State_action_list.size()<<endl;
		for (int j=0;j<Full_State_action_list.size();j++)
		{
//			cout<<Full_State_action_list[j].graph_name<<", "<<graphName<<endl;
			if (Full_State_action_list[j].graph_name==graphName)
			{
				for (int k=0;k<Full_State_action_list[j].graph_state_action_offline_vector.size();k++)
				{
//					cout<<Full_State_action_list[j].graph_state_action_offline_vector[k].state_name<<", "<<temp_obj.state_name<<endl;
					if (temp_obj.state_name==Full_State_action_list[j].graph_state_action_offline_vector[k].state_name)
					{
						nameFlag=true;
						temp_obj.actions_list=Full_State_action_list[j].graph_state_action_offline_vector[k].actions_list;

						for (int l=0;l<temp_obj.actions_list.size();l++)
						{
							vector<bool> temp_actionProgress(temp_obj.actions_list[l].assigned_agents.size(),false);
							temp_obj.actionsProgress.push_back(temp_actionProgress); //DEL
							temp_obj.actions_list[l].isDone=temp_actionProgress;
						}
						break;
					}
				}
				break;
			}
		}
		state_action_table.push_back(temp_obj);
		if(nameFlag==false)
		{
			cout<<"There is not any defined state or andor graph name with the name coming from AND/OR graph: "<<temp_obj.state_name<<", "<<graphName<<endl;
			exit(1);
		}
	}

//	cout<<"101: "<<state_action_table.size()<<endl;
	cout<<FBLU("after generate new state-action table: ")<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;

	for (int i=0;i<state_action_table.size();i++)
		state_action_table[i].PrintSummary();
//	cin>>aa;



	return CheckStateExecution();

}
void seq_planner_class::CheckStateExecution(){
	// check for all the actions in all rows of state-action table:
	// if a row is empty OR if all the actions row is done (true flag):
	//that state is solved, Delete all the vector, u
	// if there is not update for the andor graph, find the next action for the human or robot to be solved
	// in theory it can not be to different graphs at the same time have solved nodes/hyperarcs!

	cout<<BOLD(FBLU("seq_planner_class::CheckStateExecution"))<<endl;

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
			AndOrUpdateName=state_action_table[i].andorName;
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
				{
					last_action_progress=false;
					break;
				}

			if (last_action_progress==true)
			{
				updateAndor=true;
				AndOrUpdateName=state_action_table[i].andorName;
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
		return FindOptimalState();
	}
	else
	{
		timeNow=ros::Time::now().toSec();
		fileLog<< to_string(timeNow)<<" planning ended"<<"\n";
		fileLog<< to_string(timeNow)<<" andor started"<<"\n";
	}
}

void seq_planner_class::GenerateOptimalStateSimulation(void) {
	/*!
	 *	1- check for the agents to perform the action, if necessary make branches for simulation
	 *	2- Find all the parameters of the action based on state-action table (if some parameters are not assigned, we assign base on all possible parameters for that)
	 *	3- if an action could not be performed, the progress of it should be stopped for simulation.
	 *	3- if all the actions are simulated, rank them in ranking function
	 *
	 * */
	cout << BOLD(FBLU("seq_planner_class::GenerateOptimalStateSimulation" ))<< endl;
	timeNow=ros::Time::now().toSec();
	fileLog<< to_string(timeNow)<<" planning ended"<<"\n";
	fileLog<< to_string(timeNow)<<" simulation started"<<"\n";


	simulation_vector.clear();
	vector<optimal_state_simulation> temp_simulation_vector;
	// check for a filled parameters in actions of a state given by user-> if yes, give it to all the actions.
	optimal_state_simulation temp_sim;
	temp_sim.actions_list = state_action_table[optimal_state].actions_list;
	temp_sim.actionsTime.resize(temp_sim.actions_list.size(), 0.0);
	temp_sim.optimalStatePtr = &state_action_table[optimal_state];
	temp_sim.state_name = state_action_table[optimal_state].state_name;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	temp_sim.PrintSummary();

	// add the current joint values to the simulation: in order to do it I should call the knowledge base:
	knowledge_msgs::knowledgeSRV knowledge_msg_q;
	knowledge_msg_q.request.reqType="LeftArm_q";
	knowledge_msg_q.request.requestInfo="Pose";
	if (knowledgeBase_client.call(knowledge_msg_q))
	{
//		cout<<"801"<<knowledge_msg_q.response.pose.size()<<endl;
			for(int j=0;j<7;j++)
				temp_sim.simulation_q[0][j]=knowledge_msg_q.response.pose[j]; // here I have the joint values
	}
	knowledge_msg_q.request.reqType="RightArm_q";
	knowledge_msg_q.request.requestInfo="Pose";
	if (knowledgeBase_client.call(knowledge_msg_q))
	{
//		cout<<"802"<<knowledge_msg_q.response.pose.size()<<endl;
		for(int j=0;j<7;j++)
			temp_sim.simulation_q[1][j]=knowledge_msg_q.response.pose[j]; // here I have the joint values
	}


	cout<<"initial q (left Arm): ";
	for(int i=0;i<7;i++)
		cout<<temp_sim.simulation_q[0][i]<<" ";
	cout<<endl;

	cout<<"initial q (right Arm): ";
	for(int i=0;i<7;i++)
		cout<<temp_sim.simulation_q[1][i]<<" ";
	cout<<endl;

//	cout<<"803"<<endl;

	// check agents of the actions:
	// if all the agents have some assigned agents do nothing with the agents
	int agent_counter = 0;
	for (int i = 0; i < temp_sim.actions_list.size(); i++)
	{
		if (temp_sim.actions_list[i].assigned_agents[0] != "Unknown")
		{
			temp_sim.responsibleAgents =temp_sim.actions_list[i].assigned_agents;
			agent_counter++;
		}
	}

	if (agent_counter == temp_sim.actions_list.size())
	{
		cout << "all the actions have assigned agents to it" << endl;
		temp_simulation_vector.push_back(temp_sim);
	}
	else if (agent_counter == 1)
	{
		temp_sim.SetAgentForAllTheAction();
		temp_simulation_vector.push_back(temp_sim);
	}
	else if (agent_counter == 0)
	{
		cout<<"No action has an assigned agent"<<endl;
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
	}
	else
	{
		cout<< "More than one action in the state is assigned agents, please check again the state-action list"	<< endl;
		exit(1);
	}

	if(temp_simulation_vector.size()==0)
	{
		cout<<"We did not found any set of agents which can perform all the actions in the optimal state"<<endl;
		state_action_table[optimal_state].isFeasible=false;
		state_action_table[optimal_state].isSimulated=true;

		timeNow=ros::Time::now().toSec();
		fileLog<< to_string(timeNow)<<" simulation ended"<<"\n";
		fileLog<< to_string(timeNow)<<" planning started"<<"\n";

		return FindOptimalState();
	}

	//! *********************************
	/*
	 * ! Find Object types and create simulation tree branches base on that
	 */

	//! first, we find what are the predicates that we used in the simulation vector
	vector<string> sim_predicate_vec;
	for (int i = 0; i<temp_simulation_vector[0].actions_list.size(); i++)
	{
		for (int j = 0; j<temp_simulation_vector[0].actions_list[i].GeneralParameters.size(); j++)
		{
			bool the_parameter_is_found_before=false;
			if(sim_predicate_vec.size()==0)
				sim_predicate_vec.push_back(temp_simulation_vector[0].actions_list[i].GeneralParameters[j]);
			else
			{
				for (int k = 0; k<sim_predicate_vec.size(); k++)
					if(sim_predicate_vec[k]==temp_simulation_vector[0].actions_list[i].GeneralParameters[j])
					{
						the_parameter_is_found_before=true;
						break;
					}
				if(the_parameter_is_found_before==false)
					sim_predicate_vec.push_back(temp_simulation_vector[0].actions_list[i].GeneralParameters[j]);
			}

		}
	}
	cout<< "sim_predicate_vec: "<<endl;
	for(int i=0;i<sim_predicate_vec.size();i++)
		cout<< sim_predicate_vec[i]<<", ";
		cout<<endl;

	for(int h=0;h<sim_predicate_vec.size();h++)
	{

		//! Second, Check for instantiation of the parameter types:
		knowledge_msgs::knowledgeSRV knowledge_msg;
		knowledge_msg.request.reqType =sim_predicate_vec[h] ;//msg1Vector[0];//  Point1, Point2, Object: Cylinder1, Sphere2, Cone2, Plate1, connectionFrame1
		knowledge_msg.request.Name = ""; // graspingPose1, graspingPose2, CenterFrame1, ObjectFrame3 ,connectionFrame1, ...
		// if msg1.size >2 ??
		knowledge_msg.request.requestInfo = "Pose-Name"; // Pose, Pose-Name, centerPose-Name, ...
		vector<string> responseVector;


		if (knowledgeBase_client.call(knowledge_msg))
		{
			responseVector = knowledge_msg.response.names; // here It returns a vector of complete names, like: Cylinder-Cylinder1-graspingPose1, Cylinder-Cylinder1-graspingPose2, Cylinder-Cylinder2-graspingPose1
		}

		if (responseVector.size() == 0)
		{
			cout << "the knowledge base returned nothing!" << endl;
			exit(1);
		}
		else
		{
			vector<string> parameterInstantiation; // for one parameterType all the possible instantiations of it
			for (int i = 0; i < responseVector.size(); i++)// here It returns a vector of complete names, like: Cylinder-Cylinder1-graspingPose1, Cylinder-Cylinder1-graspingPose2, Cylinder-Cylinder2-graspingPose1
			{
				vector<string> parameterVector;
				boost::split(parameterVector, responseVector[i], boost::is_any_of("-"));

				for(int j = 0; j < parameterVector.size(); j++)
				{
					if(sim_predicate_vec[h]==parameterVector[j])// after the parameter type should be the parameter instantiation
					{
						if(parameterInstantiation.size()==0)
						{
							parameterInstantiation.push_back(parameterVector[j+1]);
						}
						else
						{
							bool instantiationFound=false;
							for(int k=0;k<parameterInstantiation.size();k++)
							{
								if(parameterInstantiation[k]==parameterVector[j+1])
								{
									instantiationFound=true;
									break;
								}
							}
							if(instantiationFound==false)
								parameterInstantiation.push_back(parameterVector[j+1]);
						}
					}
				}
			}
			cout<< "parameterInstantiation:( "<<sim_predicate_vec[h]<<")"<<endl;
			for(int i=0;i<parameterInstantiation.size();i++)
				cout<< parameterInstantiation[i]<<", ";
				cout<<endl;

			//! Third, Check each instantiation exist before in the parameter list of all the actions or not
			vector<string> usedParamterInstantiation;
			for(int l=0; l<parameterInstantiation.size();l++)
			{
				for (int i = 0; i<temp_simulation_vector[0].actions_list.size(); i++)
				{
					bool breakFlag=false;
					for(int j = 0; j<temp_simulation_vector[0].actions_list[i].parameterVector.size(); j++)
					{
						if(temp_simulation_vector[0].actions_list[i].parameterVector[j].ExistInstantiantionInParameter(parameterInstantiation[l])==true)
						{
							usedParamterInstantiation.push_back(parameterInstantiation[l]);
							breakFlag=true;
							break;
						}
					}
					if(breakFlag==true)
						break;
				}
			}


			cout<< "usedParamterInstantiation:( "<<sim_predicate_vec[h]<<")"<<endl;
			for(int i=0;i<usedParamterInstantiation.size();i++)
				cout<< usedParamterInstantiation[i]<<", ";
				cout<<endl;


			//! if there is not exist create branches
			//! if there exist one, give that one to all the parameters
			//! if there exit more than one do not touch them!


			if(usedParamterInstantiation.size()==0)
			{// create branches:

				vector<optimal_state_simulation> temp2_simulation_vector;
				int vecCounter=0;
				for(int l=0; l<parameterInstantiation.size();l++)
				{
					for (int m = 0; m < temp_simulation_vector.size(); m++)
					{
						temp2_simulation_vector.push_back(temp_simulation_vector[m]);
						for(int j=0;j<temp_simulation_vector[m].actions_list.size();j++)
						{
							temp2_simulation_vector[vecCounter].actions_list[j].UpdateActionAllParamters(sim_predicate_vec[h],parameterInstantiation[l]);
						}
						vecCounter++;
					}
				}
				temp_simulation_vector.clear();
				temp_simulation_vector=temp2_simulation_vector;
			}
			else if(usedParamterInstantiation.size()==1)
			{// one branch, all with the same instantiations:
				for(int i=0;i<temp_simulation_vector.size();i++)
				{
					for(int j=0;j<temp_simulation_vector[i].actions_list.size();j++)
					{
						temp_simulation_vector[i].actions_list[j].UpdateActionAllParamters(sim_predicate_vec[h],usedParamterInstantiation[0]);
					}
				}
			}
			else
			{// one branch, with different instantiations:
				cout<<"The parameter type ["<<sim_predicate_vec[h]<<"] is used with following instantiations: ";
				for(int i=0;i<usedParamterInstantiation.size();i++)
					cout<<usedParamterInstantiation[i]<<", ";
				cout<<endl;
			}

		}
	}
//
//
//		vector<optimal_state_simulation> temp2_simulation_vector;
//		for (int m = 0; m < temp_simulation_vector.size(); m++)
//		{
//			int NoAgents =temp_simulation_vector[m].responsibleAgents.size();
//			vector<string> AssignedParametersCombinations;
//			//PossibileCombinations(responseVector, NoAgents,	AssignedParametersCombinations);
//			for (int n = 0; n < responseVector.size(); 	n++)
//			{
////						cout<<"103-2: "<<responseVector[n]<<endl;
//				temp_simulation_vector[m].actions_list[i].UpdateActionParamters(responseVector[n],j);
//				temp2_simulation_vector.push_back(temp_simulation_vector[m]);
//			}
//		}
//		temp_simulation_vector.clear();
//		temp_simulation_vector=temp2_simulation_vector;
////				cout<<"103-3: "<<temp_simulation_vector.size()<<", "<<temp2_simulation_vector.size()<<endl;
//	}
//
//
//
//
//
//
////	cout<<"101"<<endl;
//
//	// check other parameters of the actions
//	// I am not sure about this part: ????
//	for (int i = 0; i<temp_simulation_vector[0].actions_list.size(); i++)
//	{
//		vector<string> parameter_type = temp_simulation_vector[0].actions_list[i].refActionDef.parameterTypes;
//		bool the_parameter_is_found_before;
//		for (int j = 0; j < parameter_type.size(); j++)
//		{
//			the_parameter_is_found_before = false;
//			for (int k = 0; k < temp_simulation_vector[0].parameters_type.size();k++)
//			{
//				if (parameter_type[j] == temp_simulation_vector[0].parameters_type[k])
//				{
//					the_parameter_is_found_before = true;
//				}
//			}
//			if (the_parameter_is_found_before == false)
//			{
//				temp_simulation_vector[0].parameters_type.push_back(parameter_type[j]);
//			}
//		}
//	}
//
//
//
//	for (int i = 1; i < temp_simulation_vector.size(); i++)
//	{
//		temp_simulation_vector[i].parameters_type =	temp_simulation_vector[0].parameters_type;
//	}
////	cout<<"103"<<endl;
//
//	// the simulation vector knows how many parameter type for the actions we need, like object grasping poses, object frame, ...
	// check for first action now how many assigned parameters it can have for each type.
//	for (int i = 0; i < state_action_table[optimal_state].actions_list.size(); 	i++)
//	{
//		for (int j = 0; j< state_action_table[optimal_state].actions_list[i].refActionDef.parameterTypes.size();j++)
//		{
////			cout<<"103-1"<<endl;
//			string actionParameterName = state_action_table[optimal_state].actions_list[i].assignedParameters[j];
//			string actionParameterType =state_action_table[optimal_state].actions_list[i].refActionDef.parameterTypes[j] +"-" +"Name";
//			//			int parameterNo;
//			//			for (int h = 0; h < temp_simulation_vector[0].parameters_type.size();h++)
//			//			{
//			//				if (temp_simulation_vector[0].parameters_type[h]== state_action_table[optimal_state].actions_list[i].refActionDef.parameterTypes[j])
//			//				{
//			//					parameterNo = h;
//			//					break;
//			//				}
//			//			}
//			vector<string> msg1Vector;
//			boost::split(msg1Vector, actionParameterName, boost::is_any_of("-"));
//			knowledge_msgs::knowledgeSRV knowledge_msg;
//			knowledge_msg.request.reqType =actionParameterName ;//msg1Vector[0];//  Point1, Point2, Object: Cylinder1, Sphere2, Cone2, Plate1, connectionFrame1
//			knowledge_msg.request.Name = ""; // graspingPose1, graspingPose2, CenterFrame1, ObjectFrame3 ,connectionFrame1, ...
//			// if msg1.size >2 ??
//			knowledge_msg.request.requestInfo = actionParameterType; // Pose, Pose-Name, centerPose-Name, ...
//			vector<string> responseVector;
//
//
//			if (knowledgeBase_client.call(knowledge_msg))
//			{
//				responseVector = knowledge_msg.response.names; // here It returns a vector of complete names, like: Cylinder-Cylinder1-graspingPose1, Cylinder-Cylinder1-graspingPose2, Cylinder-Cylinder2-graspingPose1
//				// example: graspingPose1,graspingPose2
//			}
//			if (responseVector.size() == 0)
//			{
//				cout << "the knowledge base returned nothing!" << endl;
//			}
//			else
//			{
//				vector<optimal_state_simulation> temp2_simulation_vector;
//				for (int m = 0; m < temp_simulation_vector.size(); m++)
//				{
//					int NoAgents =temp_simulation_vector[m].responsibleAgents.size();
//					vector<string> AssignedParametersCombinations;
//					//PossibileCombinations(responseVector, NoAgents,	AssignedParametersCombinations);
//					for (int n = 0; n < responseVector.size(); 	n++)
//					{
////						cout<<"103-2: "<<responseVector[n]<<endl;
//						temp_simulation_vector[m].actions_list[i].UpdateActionParamters(responseVector[n],j);
//						temp2_simulation_vector.push_back(temp_simulation_vector[m]);
//					}
//				}
//				temp_simulation_vector.clear();
//				temp_simulation_vector=temp2_simulation_vector;
////				cout<<"103-3: "<<temp_simulation_vector.size()<<", "<<temp2_simulation_vector.size()<<endl;
//			}
//		}
//	}


	simulation_vector=temp_simulation_vector;
	cout<<"simulation vector: (size: "<<simulation_vector.size()<<")"<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	for(int i=0;i<simulation_vector.size();i++)
		simulation_vector[i].PrintSummary();



	if(simulation_vector.size()==0)
	{
		state_action_table[optimal_state].isFeasible=false;
		state_action_table[optimal_state].isSimulated=true;
		cout<<" Error: The Simulation Vector is Empty"<<endl;

		timeNow=ros::Time::now().toSec();
		fileLog<< to_string(timeNow)<<" simulation ended"<<"\n";
		fileLog<< to_string(timeNow)<<" planning started"<<"\n";

		return FindOptimalState();

	}
	else
	{
		return GiveSimulationCommand();
	}


	// give simulation command to the robot interface:

}

void seq_planner_class::GiveSimulationCommand(void){
	// 1- find the first command it should publish
	// 2- Fill the msg for giving the command
	// 3- publish the command
	cout<<(FBLU("seq_planner_class::GiveSimulationCommand"))<<endl;
//	for(int i=0;i<simulation_vector.size();i++)
//		simulation_vector[i].Print();


	bool breakFlag=false;
//	cout<<"201"<<endl;
	//	find the first command it should publish, or the first action that is not solved
	for(int i=0;i< simulation_vector[0].actions_list.size();i++)
	{
		for(int j=0;j< simulation_vector.size();j++)
		{
//			cout<<"201-0-"<<j<<i<<endl;
			if(simulation_vector[j].actions_list[i].isDone[0]==false)
			{
//				cout<<"201-1-"<<j<<i<<endl;
				simulationVectorNumber=j;
				SimulationActionNumber=i;
				breakFlag=true;
				break;
			}
		}
		if(breakFlag==true)
			break;
	}

	if(breakFlag==false)
		return RankSimulation();

//	cout<<"202: "<<simulationVectorNumber<<" "<<SimulationActionNumber<<endl;
	vector<string> emptyvec;
	if(CanAgentPerformAction(emptyvec,"Human",simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].name,true ))
	{
		for(int i=0;i<simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].isDone.size();i++)
			simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].isDone[i]= true;

		simulation_vector[simulationVectorNumber].actionsTime[SimulationActionNumber]=0.0;

		if(simulationVectorNumber==(int)simulation_vector.size()-1
				&& SimulationActionNumber==(int)simulation_vector[simulationVectorNumber].actions_list.size()-1 )
		{
			cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
			simulation_vector[simulationVectorNumber].PrintSummary();
			return RankSimulation();
		}
		else

			return GiveSimulationCommand();
	}

	robot_interface_msgs::SimulationRequestMsg req_instance;

	// give the current simulation q to the msg
	for(int i=0; i<2;i++)
	{
		robot_interface_msgs::Joints joint_values;
		for(int j=0;j<7;j++)
		{
			joint_values.values.push_back(simulation_vector[simulationVectorNumber].simulation_q[i][j]);
		}
		req_instance.ArmsJoint.push_back(joint_values);
	}
//	cout<<"203"<<endl;

	// add actions name to the msg
	req_instance.ActionName=simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].name;


	// add actions parameter names +info to the msg
	for (int i=0;i<simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].assignedParameters.size();i++)
	{
		req_instance.ActionParametersName.push_back(simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].assignedParameters[i]);
		req_instance.ActionParameterInfo.push_back( simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].refActionDef.parameterTypes[i]);
	}
//	cout<<"204"<<endl;

	//	// add responsible agents to the msg
	vector<string> temp_colleagues;
	for (int i=0;i<simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].assigned_agents.size();i++)
	{
		if( ResponsibleAgentType(simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].assigned_agents[i])=="Human" )
			req_instance.ColleagueAgents.push_back(simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].assigned_agents[i]);
		else
			req_instance.ResponsibleAgents.push_back(simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].assigned_agents[i]);
	}
//	cout<<"205"<<endl;

	pubSimulationCommand.publish(req_instance);
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	cout<<req_instance.ActionName<<", ";
	for(int i=0;i<req_instance.ActionParametersName.size();i++)
		cout<<req_instance.ActionParametersName[i]<<" , ";

	for(int i=0;i<req_instance.ResponsibleAgents.size();i++)
		cout<<req_instance.ResponsibleAgents[i]<<", ";

	for(int i=0;i<req_instance.ColleagueAgents.size();i++)
		cout<<"("<<req_instance.ColleagueAgents[i]<<"), ";
	cout<<endl;
	cout<<"q Left: ";
	for(int i=0;i<req_instance.ArmsJoint[0].values.size();i++)
		cout<<req_instance.ArmsJoint[0].values[i]<<" ";
	cout<<endl;
	cout<<"q Right: ";
	for(int i=0;i<req_instance.ArmsJoint[1].values.size();i++)
		cout<<req_instance.ArmsJoint[1].values[i]<<" ";
	cout<<endl;
	cout<<"*******************"<<endl;
}

void seq_planner_class::UpdateSimulation(const robot_interface_msgs::SimulationResponseMsg&  simulationResponse){

	// 1- Fill the necessary data inside the simulation vector
	// 2- if an action is done completely, fill also all the consequential action from other simulation vector
	// 3- if all the actions in all the simulation vectors is done, go to the RankSimulation Functions, otherwise go to the GiveCommand Function
	cout<<(FBLU("seq_planner_class::UpdateSimulation"))<<endl;

	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	cout<<"time: "<<simulationResponse.time<<", ";
	cout<<"success: "<<(bool)simulationResponse.success <<": ";

	cout<<simulationResponse.ActionName<<", ";
	for(int i=0;i<simulationResponse.ActionParametersName.size();i++)
		cout<<simulationResponse.ActionParametersName[i]<<", ";

	for(int i=0;i<simulationResponse.ResponsibleAgents.size();i++)
		cout<<simulationResponse.ResponsibleAgents[i]<<", ";

	for(int i=0;i<simulationResponse.ColleagueAgents.size();i++)
		cout<<"("<<simulationResponse.ColleagueAgents[i]<<"), ";
	cout<<endl;
	cout<<"q Left: ";
	for(int i=0;i<simulationResponse.ArmsJoint[0].values.size();i++)
		cout<<simulationResponse.ArmsJoint[0].values[i]<<" ";
	cout<<endl;
	cout<<"q Right: ";
	for(int i=0;i<simulationResponse.ArmsJoint[1].values.size();i++)
		cout<<simulationResponse.ArmsJoint[1].values[i]<<" ";
	cout<<endl;

	cout<<"*******************"<<endl;
	vector<bool> qChangeByAction={false, false};
	vector<double> q_old(7,0.0),q_new(7,0.0);
	bool failure_Flag=false;
	for(int i=0;i< simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].assigned_agents.size();i++)
	{
		for(int j=0; j< simulationResponse.ResponsibleAgents.size();j++)
		{
//			cout<<"301-0: "<<simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].assigned_agents[i]<<", "<<(string)simulationResponse.ResponsibleAgents[j]<<endl;
			if(simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].assigned_agents[i]== (string)simulationResponse.ResponsibleAgents[j])
			{
//				cout<<"301-1"<<endl;
				if((bool)simulationResponse.success==true)
				{
//					cout<<"301-2"<<endl;

					simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].isDone[i]=true;
					// fill the simulation timing:
					if((double)simulationResponse.time>simulation_vector[simulationVectorNumber].actionsTime[SimulationActionNumber])
						simulation_vector[simulationVectorNumber].actionsTime[SimulationActionNumber]=simulationResponse.time;

					for(int k=0;k<7;k++)
					{
						q_old[i]=simulation_vector[simulationVectorNumber].simulation_q[0][k];
						q_new[i]=simulationResponse.ArmsJoint[0].values[k];
					}
					qChangeByAction[0]=ActionChangesRobot_q(q_old,q_new);

					for(int k=0;k<7;k++)
					{
						q_old[i]=simulation_vector[simulationVectorNumber].simulation_q[1][k];
						q_new[i]=simulationResponse.ArmsJoint[1].values[k];
					}
					qChangeByAction[1]=ActionChangesRobot_q(q_old,q_new);

					// Fill the simulation joint values:
					if((string)simulationResponse.ResponsibleAgents[j]=="LeftArm")
					{
//						cout<<"301-3"<<endl;
						for(int k=0;k<7;k++)
							simulation_vector[simulationVectorNumber].simulation_q[0][k]=simulationResponse.ArmsJoint[0].values[k];
					}
					else if((string)simulationResponse.ResponsibleAgents[j]=="RightArm")
					{
//						cout<<"301-4"<<endl;
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
//					cout<<"301-5"<<endl;
					failure_Flag=true;
				}
				break;
			}
		}
		if(failure_Flag==true)
			break;
	}
//	cout<<"302"<<endl;

	// if the simulation shows failure:
	// the action can not be performed, Delete that simulation and all the other similar ones from simulation vector
	// give a new simulation command to the robot
	if(failure_Flag==true)
	{
		vector<optimal_state_simulation>::iterator it = (simulation_vector.begin()+simulationVectorNumber);
		cout<<"*** vector to Delete: *** "<<endl;
		cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
		(*it).PrintSummary();
		simulation_vector.erase(it);


		/*

		action temp_action_for_DEL(simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber]);
		for(vector<optimal_state_simulation>::iterator it =simulation_vector.begin(); it!= simulation_vector.end();)
		{
//			temp_action_for_DEL.Print();
//			(*it).actions_list[SimulationActionNumber].Print();
			if(isTwoActionsEqual(temp_action_for_DEL,(*it).actions_list[SimulationActionNumber] ))
			{
//				cout<<"302-2"<<endl;
				simulation_vector.erase(it);
			}
			else
			{it++;}
		}
		//GiveSimulationCommand(); // should be given later not here

		*/
	}
	else // the simulation shows successful implementation, give the bool to all the actions with exactly the same parameters:
	{
	/*
//		cout<<"302-3-"<<simulation_vector.size()<<endl;
		bool actionCompletelydone=true;
		for(int i=0; i<simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].isDone.size();i++)
			if(simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].isDone[i]==false)
				actionCompletelydone=false;

		if(actionCompletelydone==true)
		{
//			cout<<"302-4-"<<simulation_vector.size()<<endl;
			for(vector<optimal_state_simulation>::iterator it =simulation_vector.begin(); it!= simulation_vector.end();it++)
				if(isTwoActionsEqual(simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber],(*it).actions_list[SimulationActionNumber] ))
				{
//					simulation_vector[simulationVectorNumber].actions_list[SimulationActionNumber].Print();
//					(*it).actions_list[SimulationActionNumber].Print();

					for(int j=0;j< (*it).actions_list[SimulationActionNumber].isDone.size();j++)
					{
//						cout<<"302-5-"<<j<<endl;
						(*it).actions_list[SimulationActionNumber].isDone[j]=true; // we are sure all of them is true
						(*it).actionsTime[SimulationActionNumber]=simulation_vector[simulationVectorNumber].actionsTime[SimulationActionNumber];

						if((*it).actions_list[SimulationActionNumber].assigned_agents[j]=="LeftArm" && qChangeByAction[0]==true)
						{
							for(int k=0;k<7;k++)
								(*it).simulation_q[0][k]=simulation_vector[simulationVectorNumber].simulation_q[0][k];
						}
						else if((*it).actions_list[SimulationActionNumber].assigned_agents[j]=="RightArm" && qChangeByAction[1]==true)
						{
							for(int k=0;k<7;k++)
								(*it).simulation_q[1][k]=simulation_vector[simulationVectorNumber].simulation_q[1][k];
						}
						else{}
					}
				}
		}
		*/
	}
//	cout<<"303"<<endl;
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

	cout<<"simulation vector size: "<<simulation_vector.size()<<endl;

//	for(int i=0;i<simulation_vector.size();i++)
//		simulation_vector[i].PrintSummary();

//	cout<<"304-"<<OptimalStateCompletelySimulated<<endl;
	if(OptimalStateCompletelySimulated==true)
		return RankSimulation();
	else
		return GiveSimulationCommand();

}

void seq_planner_class::RankSimulation(void){

	//! check if the simulation vector is not empty:
	cout<<BOLD(FBLU("seq_planner_class::RankSimulation"))<<endl;

	cout<<"simulation vector size: "<<simulation_vector.size()<<endl;
	cout<<FBLU("Time: ")<<to_string(ros::Time::now().toSec())<<endl;
	for(int i=0;i<simulation_vector.size();i++)
		simulation_vector[i].PrintSummary();

	if(simulation_vector.size()==0)
	{
//		cout<<"401-1"<<endl;
		state_action_table[optimal_state].isFeasible=false;
		state_action_table[optimal_state].isSimulated=true;

		timeNow=ros::Time::now().toSec();
		fileLog<< to_string(timeNow)<<" simulation ended"<<"\n";
		fileLog<< to_string(timeNow)<<" planning started"<<"\n";

		return FindOptimalState();
	}


//	cout<<"402"<<endl;
	//! find the total cost for all of the simulation
	for(int i=0;i<simulation_vector.size();i++)
	{
		for(int j=0;j<simulation_vector[i].actionsTime.size();j++)
			simulation_vector[i].total_cost=simulation_vector[i].total_cost+simulation_vector[i].actionsTime[j];
	}
	cout<<"total simulation costs: ";
	for(int i=0;i<simulation_vector.size();i++)
		cout<<simulation_vector[i].total_cost<<", ";
	cout<<endl;

	//! Find the minimum cost and respecting index of it:
	double minCost=100000000.0;
	int minIndex=0;
//	cout<<"403"<<endl;
	for(int i=0;i<simulation_vector.size();i++)
		if(simulation_vector[i].total_cost<minCost)
		{
			minIndex=i;
			minCost=simulation_vector[i].total_cost;
		}

	//! give the parameters of the simulation to the optimal state and find the next action
//	cout<<"404"<<endl;
	state_action_table[optimal_state].isSimulated=true;
	state_action_table[optimal_state].isFeasible=true;

	for(int i=0;i<state_action_table[optimal_state].actions_list.size();i++)
	{
		state_action_table[optimal_state].actions_list[i].assigned_agents=simulation_vector[minIndex].actions_list[i].assigned_agents;
		state_action_table[optimal_state].actions_list[i].assignedParameters=simulation_vector[minIndex].actions_list[i].assignedParameters;

		string temp_action_parameters;
		temp_action_parameters=state_action_table[optimal_state].actions_list[i].name;
		for(int j=0;j<state_action_table[optimal_state].actions_list[i].assignedParameters.size();j++)
		{
			temp_action_parameters=temp_action_parameters+"_"+state_action_table[optimal_state].actions_list[i].assignedParameters[j];
		}
		state_action_table[optimal_state].actions_list[i].actionAndParameters=temp_action_parameters;

	}
	timeNow=ros::Time::now().toSec();
	fileLog<< to_string(timeNow)<<" simulation ended"<<"\n";
	fileLog<< to_string(timeNow)<<" planning started"<<"\n";

	return FindNextAction();
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

	//	cout<<file_path_ifStr.is_open()<<endl;
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
				if(actionDef.actionType=="simple")
				{}
				else if(actionDef.actionType=="complex")
				{
					complexActionsList.push_back(actionDef.name);
					//SetStateActionList(seq_planner_path+"/StateActionList_"+actionDef.name+".txt",actionDef.ComplexAction_state_action_list);
				}
				else
				{
					cout<<"Error in action type in action definition list: action name: "<<actionDef.name<<", action type"<<actionDef.actionType<<endl;
				}


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
	else
	{
		cout<<"The file can not open: "<<actionDefinitionPath<<endl;
		exit(1);
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

	//	for(int k=0;k<agents.size();k++)
	//		agents[k].Print();
	//	cout<<"************************"<<endl;

}

void seq_planner_class::SetStateActionList(string stateActionPath, string andorGraphName, vector<offline_state_action> & offline_state_action_list){
	cout<<"seq_planner_class::SetStateActionList"<<endl;

	ifstream file_path_ifStr(stateActionPath.c_str());
	vector<string> line_list;


	string line;
	string delim_type=" ";
	string responsible_delim_type="?";
	string jointAction_delim_type="+";
	string actionParameter_delim_type="_";
	string firstPartOfParamter_delim_type="-";


	if (file_path_ifStr.is_open())
	{
		while(getline(file_path_ifStr,line))
		{
			offline_state_action temp_state_action;
			boost::split(line_list, line, boost::is_any_of(delim_type));
			temp_state_action.state_name=line_list[0];
			//			temp_state_action.andorName=andorGraphName;
			for(int i=1;i<line_list.size();i++)
			{
				vector<string> action_and_responsibles,action_and_parameters;
				vector<string> responsibles;

				boost::split(action_and_responsibles, line_list[i], boost::is_any_of(responsible_delim_type));
				boost::split(action_and_parameters, action_and_responsibles[0], boost::is_any_of(actionParameter_delim_type));
				bool actionName=false;
				int actionRefIndex;
				for(int j=0;j<action_Definition_List.size();j++)
					if(action_and_parameters[0]==action_Definition_List[j].name)
					{
						actionName=true;
						actionRefIndex=j;
						break;
					}
				if(actionName==false){
					cout<<"The written action is not defined in actions definition list, please check: state: "<<line_list[0]<<", action: "<<action_and_parameters[0]<<endl;
					exit(1);
				}
				action tempAction(action_Definition_List[actionRefIndex]);

				tempAction.actionAndParameters=action_and_responsibles[0];


				for(int j=1;j<action_and_parameters.size();j++)
				{
					tempAction.assignedParameters.push_back(action_and_parameters[j]);

					vector<string> firstPartofParameter;
					boost::split(firstPartofParameter, action_and_parameters[j], boost::is_any_of(firstPartOfParamter_delim_type));
					paramter tempParameter;

					for(int k=0;k<firstPartofParameter.size();k++)
					{
						;
						bool paramIsPredicate=false;
						for(int m=0;m<objectTypeVector.size();m++)
							if(firstPartofParameter[k]==objectTypeVector[m])
							{
								paramIsPredicate=true;
								tempAction.GeneralParameters.push_back(firstPartofParameter[k]);
								break;
							}

						tempParameter.paramVec.push_back(firstPartofParameter[k]);
						if(paramIsPredicate==true)
						{
							tempParameter.parameterType.push_back(eParamType::predicate);
						}
						else
						{
							tempParameter.parameterType.push_back(eParamType::instantiation);
						}
					}
					tempAction.parameterVector.push_back(tempParameter);
				}

				tempAction.Action_GeneralParameters=tempAction.name;
				for(int j=0;j<tempAction.GeneralParameters.size();j++)
				{
					tempAction.Action_GeneralParameters+="_"+tempAction.GeneralParameters[j];
				}

				if(action_and_responsibles.size()==2)
				{
					boost::split(responsibles, action_and_responsibles[1], boost::is_any_of(jointAction_delim_type));
					tempAction.assigned_agents=responsibles;
				}
				else if(action_and_responsibles.size()==1)
				{
					responsibles.push_back("Unknown");
					tempAction.assigned_agents=responsibles;
				}
				else
				{
					cout<<"Error in state_action text file, please check state: "<<temp_state_action.state_name<<", action: "<<action_and_responsibles[0]<<endl;
				}
				tempAction.isDone.resize(responsibles.size(),false);


				temp_state_action.actions_list.push_back(tempAction);

			}
			offline_state_action_list.push_back(temp_state_action);


		}
		file_path_ifStr.close();
	}

}

void seq_planner_class::ReadObjectsType(string objTypePath){
	cout<<"seq_planner_class::ReadObjectsType"<<endl;



	ifstream file_path_ifStr(objTypePath.c_str());

	string line;

	//	cout<<file_path_ifStr.is_open()<<endl;
	if (file_path_ifStr.is_open())
	{
		while(getline(file_path_ifStr,line))
		{
			objectTypeVector.push_back(line);
		}
	}
};

void seq_planner_class::CheckStateActionList(){
	cout<<"seq_planner_class::CheckStateActionList"<<endl;
	bool isResponsibleAgentAcceptable;

	for(int i=0;i<Full_State_action_list.size();i++)
	{
		for(int j=0;j<Full_State_action_list[i].graph_state_action_offline_vector.size();j++)
		{
			for(int k=0;k<Full_State_action_list[i].graph_state_action_offline_vector[j].actions_list.size();k++)
			{
				isResponsibleAgentAcceptable=false;
				if(Full_State_action_list[i].graph_state_action_offline_vector[j].actions_list[k].assigned_agents[0]=="Unknown")
				{
					isResponsibleAgentAcceptable=true;
					// if the action have one set of agents to perfrom assign it here:
					if(Full_State_action_list[i].graph_state_action_offline_vector[j].actions_list[k].refActionDef.possible_agents.size()==1)
					{
						Full_State_action_list[i].graph_state_action_offline_vector[j].actions_list[k].assigned_agents=Full_State_action_list[i].graph_state_action_offline_vector[j].actions_list[k].refActionDef.possible_agents[0];
					}
				}
				else
				{
					isResponsibleAgentAcceptable=CanAgentPerformAction(Full_State_action_list[i].graph_state_action_offline_vector[j].actions_list[k].assigned_agents,"",Full_State_action_list[i].graph_state_action_offline_vector[j].actions_list[k].name, true);
				}

				if(isResponsibleAgentAcceptable==false)
				{
					cout<<FRED("The agent you defined in the 'Full_State_action_list' file can not perform the given action: state:")<<
							Full_State_action_list[i].graph_state_action_offline_vector[j].state_name<<", action:"<<Full_State_action_list[i].graph_state_action_offline_vector[j].actions_list[k].name<<endl;
					cout<<"Do you want to assign a new agent to it? (enter 1 if yes)";
					bool input;
					vector<string> temp_agent_list;

					cin>>input;
					if(input==true)
					{
						cout<<"Give one of the following rows as responsible:"<<endl;
						for(int m=0;m<Full_State_action_list[i].graph_state_action_offline_vector[j].actions_list[k].refActionDef.possible_agents.size();m++)
						{
							for(int n=0;n<Full_State_action_list[i].graph_state_action_offline_vector[j].actions_list[k].refActionDef.possible_agents[m].size();n++)
							{
								cout<<Full_State_action_list[i].graph_state_action_offline_vector[j].actions_list[k].refActionDef.possible_agents[m][n];
								if(n<Full_State_action_list[i].graph_state_action_offline_vector[j].actions_list[k].refActionDef.possible_agents[m].size()-1)
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
					Full_State_action_list[i].graph_state_action_offline_vector[j].actions_list[k].assigned_agents=temp_agent_list;
				}
			}
		}
	}
};



bool seq_planner_class::CanAgentPerformAction(vector<string> agent_name, string agent_type, string action_name, bool sufficiency){
//	cout<<"seq_planner_class::CanAgentPerformAction"<<endl;
//	cout<<"actions_name: "<<action_name<<", agent_type: "<<agent_type<<", agent_name: ";
//	for (int i=0;i<agent_name.size();i++)
//		cout<<agent_name[i]<<" ";
//	cout<<endl;

	bool temp=false;
	int action_number=1000;
	if(agent_name.empty()!=true && agent_type=="")// if a specific agent (by name) can perform an action
	{
		for (int i=0;i<action_Definition_List.size();i++)
		{
			if(action_Definition_List[i].name==action_name)
			{
				action_number=i;
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
			for(int l=0;l<action_Definition_List[action_number].possible_agents[j].size();l++)
			{
				for(int k=0;k<agent_name.size();k++)
				{
					if(action_Definition_List[action_number].possible_agents[j][l]==agent_name[k])
					{
						count++;
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
		for (int i=0;i<action_Definition_List.size();i++)
		{
			if(action_Definition_List[i].name==action_name)
			{
				action_number=i;
				break;
			}
		}
		for(int j=0;j<action_Definition_List[action_number].possible_agents.size();j++)
		{
			for(int l=0; l< action_Definition_List[action_number].possible_agents[j].size();l++)
			{
				for(int k=0; k<agents.size(); k++)
				{
					if(action_Definition_List[action_number].possible_agents[j][l]==agents[k].name)
					{
						if(agents[k].type==agent_type)
						{
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

//	cout<<" Can agent perform the action? "<<temp<<endl;
	return temp;
};

void seq_planner_class::EmergencyRobotStop(void){
	cout<<BOLD(FBLU("seq_planner_class::EmergencyRobotStop"))<<endl;
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

//				agents[i].lastAssignedAction="UnGrasp";
//				PublishRobotAction("UnGrasp", agentName, coleaguesName);
			}
//			else if(agents[i].lastAssignedAction=="UnGrasp")
//			{
//				agents[i].lastAssignedAction="Rest";
//				PublishRobotAction("Rest", agentName, coleaguesName);
//			}
//			else if(agents[i].lastAssignedAction=="Rest")
//			{
//				agents[i].lastAssignedAction="HoldOn";
//				PublishRobotAction("HoldOn", agentName, coleaguesName);
//			}
			else
			{
				agents[i].lastAssignedAction="Stop";
				PublishRobotAction("Stop", agentName, coleaguesName);
				PublishRobotAction("UnGrasp", agentName, coleaguesName);
//				usleep(0.5e6);
				PublishRobotAction("Rest", agentName, coleaguesName);
			}
		}

	}
};

void seq_planner_class::UpdateRobotEmergencyFlag(string ActionName, vector<string>AgentsName, bool success){
	cout<<BOLD(FBLU("seq_planner_class::UpdateRobotEmergencyFlag"))<<endl;

	for(int i=0;i<agents.size();i++)
	{
		if(agents[i].name==AgentsName[0])
		{
//			cout<<"800"<<endl;
			agents[i].isBusy=false;
			agents[i].isSuccessfullyDone=success;
			agents[i].lastActionAck=ActionName;
			agents[i].Print();
		}
	}

	vector<bool> emergencyFlagVector, holdOnFlag;
//	cout<<"*****************800-1**************"<<endl;
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
//				cout<<"803"<<endl;
			}
			else if(agents[i].isBusy==false && agents[i].isSuccessfullyDone==true &&  agents[i].lastActionAck=="HoldOn" && agents[i].lastAssignedAction=="HoldOn")
			{
				emergencyFlagVector[count]=true;
//				cout<<"804"<<endl;
			}
			else
			{}
			count++;
		}
	}
	bool temp_EmergencyFlag=true, temp_HoldOnFlag=true;
	count=0;
//	cout<<"*****************800-2**************"<<endl;
	for(int i=0;i<agents.size();i++)
	{
		if(agents[i].type=="Robot")
		{
			agents[i].Print();
			if(holdOnFlag[count]==false)
			{
//				cout<<"*****"<<805<<endl;
				temp_HoldOnFlag=false;


			}
			if(emergencyFlagVector[count]==false)
			{
//				cout<<"*****"<<806<<endl;
				temp_EmergencyFlag=false;
			}
			count++;
		}
	}

//	cout<<"Updated hold on Flag: "<<temp_HoldOnFlag<<endl;
//	cout<<"Updated Emergency Flag: "<<temp_EmergencyFlag<<endl;

	if(temp_HoldOnFlag==true)
		EmergencyRobotStop();
	else if(temp_EmergencyFlag==true)
		emergencyFlag=!temp_EmergencyFlag;
	else
	{}

//	cout<<"emergency Flag: "<<emergencyFlag<<endl;
};


void seq_planner_class::CallBackHumanAck(const std_msgs::String::ConstPtr& msg){
	cout<<BOLD(FBLU("seq_planner_class::CallBackHumanAck"))<<endl;

	string actionName=msg-> data.c_str();
	cout<<agents[0].name<<" action: "<< actionName<<endl;
	if(agents[0].isBusy==false)
	{
		// it means that, beforehand no action is assigned to human action
		timeNow=ros::Time::now().toSec();
		fileLog<< to_string(timeNow)<<" planning ended"<<"\n";
		fileLog<< to_string(timeNow)<<" HumanActionEmergency started"<<"\n";

		EmergencyRobotStop();
	}
	timeNow=ros::Time::now().toSec();
	fileLog<< to_string(timeNow)<<" HumanAction ended"<<"\n";
	fileLog<< to_string(timeNow)<<" planning started"<<"\n";


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
	cout<<BOLD(FBLU("seq_planner_class::CallBackRobotAck"))<<endl;
	if(emergencyFlag==false)
	{
		timeNow=ros::Time::now().toSec();
		fileLog<< to_string(timeNow)<<" RobotAction ended"<<"\n";
		fileLog<< to_string(timeNow)<<" planning started"<<"\n";
	}

	vector<string> agentsName, actionAndParametersVec;
	vector<string> robot_ack_list;
	bool success;
	string delim_type=" ", agents_delim_type="+",actionParameters_delim_type="_";
	string robot_ack, actionAndParameters, ActionAndGeneralParameters;
	bool arrived_msg=true;

	ROS_INFO("I heard Robot Ack: [%s]", msg->data.c_str());

	robot_ack=msg->data.c_str();
	// msg= actionName_Parameters responsibleAgents true/false: Approach_Point-0 LeftArm true
	boost::split(robot_ack_list, robot_ack, boost::is_any_of(delim_type));

	if(robot_ack_list.size()!=3)
	{
		cout<<"Error In receiving msg from robot ack , msg size error"<<endl;
		arrived_msg=false;
	}

	actionAndParameters=robot_ack_list[0];
	boost::split(actionAndParametersVec, actionAndParameters, boost::is_any_of(actionParameters_delim_type));// 0: action name, 1,... : parameters name

	ActionAndGeneralParameters+=actionAndParametersVec[0];
	for(int i=1;i<actionAndParametersVec.size();i++)
	{
		vector<string> parameter;
		boost::split(parameter, actionAndParametersVec[i], boost::is_any_of("-"));// Point0, Cylinder1-GraspingPose1
		ActionAndGeneralParameters+="_"+parameter[0];
	}

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

//	cout<<"emergency Flag: "<<emergencyFlag<<endl;
	if(CanAgentPerformAction(agentsName,"",actionAndParametersVec[0],false)==true && arrived_msg==true)
		if(emergencyFlag==false)
		{
			UpdateStateActionTable(actionAndParameters,agentsName,success);
		}
		else
		{
			UpdateRobotEmergencyFlag(actionAndParameters,agentsName,success);
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
	cout<<BOLD(FBLU("seq_planner_class::PublishHumanAction"))<<endl;
	cout<<AgentName<<FBLU(": Please perform Action: ")<<ActionName<<" ";

	std_msgs::String msgToDisplay;
	msgToDisplay.data=(ActionName+" Human").c_str();
	pubToRobotDisplay.publish(msgToDisplay);

	timeNow=ros::Time::now().toSec();
	fileLog<< to_string(timeNow)<<" planning ended"<<"\n";
	fileLog<< to_string(timeNow)<<" HumanAction started"<<"\n";

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
	cout<<BOLD(FBLU("seq_planner_class::PublishRobotAction"))<<endl;
	// publish which agent to perform an action

	timeNow=ros::Time::now().toSec();
	fileLog<< to_string(timeNow)<<" planning ended"<<"\n";
	fileLog<< to_string(timeNow)<<" RobotAction started"<<"\n";

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

	pubToRobotDisplay.publish(robotMsg);


	ROS_INFO("publish robot command : %s ",robotMsg.data.c_str() );
}

bool seq_planner_class::ActionChangesRobot_q(vector<double> &q_old, vector<double> &q_new ){
	double distanceThreshold=0.1, distance=0.0;

	for(int i=0;i<q_old.size();i++)
		distance+=abs(q_old[i]-q_new[i]);

	if(distance<distanceThreshold)
		return true;
	else
		return false;

};
