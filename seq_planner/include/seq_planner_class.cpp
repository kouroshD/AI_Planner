#include "seq_planner_class.hpp"

seq_planner_class::seq_planner_class(string actionDefinitionPath,string stateActionPath){
	optimal_state=0;
//	actionList=NULL;

	SetActionDefinitionList(actionDefinitionPath);
	SetStateActionList( stateActionPath);
	updateAndor=true;
	nodeSolved=false;
	haSolved=false;

}
void seq_planner_class::GenerateStateActionTable(vector<vector<string>> gen_Feasible_state_list, vector<int> gen_Feasible_stateCost_list){

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

				for (int k=0;k<Full_State_action_list[j].size();k++)
				{
					Feasible_State_action_row.push_back(Full_State_action_list[j][k]);
					Feasible_states_actions_progress_row.push_back(false);

				}
				Feasible_states_actions_table.push_back(Feasible_State_action_row);
				Feasible_states_actions_progress.push_back(Feasible_states_actions_progress_row);
			}
		}
	}
	CheckStateActionTable();

}
void seq_planner_class::CheckStateActionTable(){
// check for all the actions in all rows of state-action table:
	// if a row is empty OR if all the actions row is done (true flag):
		//that state is solved, Delete all the vector, u

	if(!Solved_node_list.empty() || !Solved_hyperarc_list.empty()){
		cout<<FRED("The solve nodes or hyperarc lists are not empty!" )<<endl;
	}

	for(int i=0;i<Feasible_states_actions_progress.size();i++)
	{
		if (Feasible_states_actions_progress[i].size()==0)
		{
			updateAndor=true;
			if (Feasible_states_Names[i][1]=="Node")
			{
				Solved_node_list.push_back(Feasible_states_Names[i][0]);
				nodeSolved=true;
			}
			if (Feasible_states_Names[i][2]=="Hyperarc")
			{
				Solved_hyperarc_list.push_back(Feasible_states_Names[i][0]);
				haSolved=true;
			}
		}
		else
		{
			int counter=0;
			for(int j=0;j<Feasible_states_actions_progress[i].size();j++)
			{
				if (Feasible_states_actions_progress[i][j]==true)
				{
					counter++;
				}
			}
			if (Feasible_states_actions_progress[i].size()==counter)
			{
				updateAndor=true;
				if (Feasible_states_Names[i][1]=="Node")
				{
					Solved_node_list.push_back(Feasible_states_Names[i][0]);
					nodeSolved=true;
				}
				if (Feasible_states_Names[i][2]=="Hyperarc")
				{
					Solved_hyperarc_list.push_back(Feasible_states_Names[i][0]);
					haSolved=true;
				}
			}
		}
	}
}


void seq_planner_class::SetFeasibleStates(void){

}

void seq_planner_class::UpdateActionStateTable(void){

}

void seq_planner_class::CallBackHumanAck(void){

}

void seq_planner_class::CallBackRobotAck(void){

}

void seq_planner_class::SetActionDefinitionList(string actionDefinitionPath){

//	cout<<"seq_planner_class::SetActionList"<<endl;
//	ROS_INFO("%s",actionDefinitionPath.c_str());
    ifstream file_path_ifStr(actionDefinitionPath.c_str());
    string delim_type=" ";
    std::vector<std::string> line_list;
    string line;
    action actionDef;
//    cout<<file_path_ifStr.is_open()<<endl;
    if (file_path_ifStr.is_open()){
    	while(getline(file_path_ifStr,line)){
    		boost::split(line_list, line, boost::is_any_of(delim_type));
//    		cout<<line<<endl;
//    		cout<<line_list<<endl;
//    		cout<<"***"<<endl;
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
void seq_planner_class::SetStateActionList(string stateActionPath){
		cout<<"SetStateActionList"<<endl;
	    ifstream file_path_ifStr(stateActionPath.c_str());
	    std::vector<std::string> line_list;
	    string line;
	    string delim_type=" ";
	    if (file_path_ifStr.is_open()){
	    	while(getline(file_path_ifStr,line)){
	    		boost::split(line_list, line, boost::is_any_of(delim_type));
	    		Full_State_action_list.push_back(line_list);
	    	}
	    	file_path_ifStr.close();
	    }

	    for (int m=0;m<Full_State_action_list.size();m++){
	    	for (int n=0;n<Full_State_action_list[m].size();n++)
	    	cout<<Full_State_action_list[m][n]<<" ";
	    	cout<<endl;
	    }

}

seq_planner_class::~seq_planner_class(){


}


