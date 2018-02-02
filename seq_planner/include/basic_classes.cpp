#include "basic_classes.hpp"
using namespace std;

actionDef::actionDef(void){
	name="";
	actionType="";
	actionMode="single";
};
actionDef::~actionDef(){};
void actionDef::Print(void){
	cout<<">>>>>>>> actionDef info"<<endl;
	cout<<"name: "<<name<<endl;
	cout<<"actionType: "<<actionType<<endl;
	cout<<"actionMode: "<<actionMode<<endl;
	cout<<"possible Agents: ";
	for (int i=0;i<possible_agents.size();i++)
	{
		for (int j=0; j<possible_agents[i].size();j++)
			cout<<possible_agents[i][j]<<" ";
		cout<<" | ";
	}
	cout<<endl;

	cout<<"Action Argument Feature: ";
	for (int i=0;i<parameterTypes.size();i++)
	{
		cout<<parameterTypes[i]<<" ";
	}
	cout<<endl;

};
/////////////////////////////////////////////////////////////

action::action(actionDef &actionDefObj):refActionDef(actionDefObj){
	name=refActionDef.name;
	actionAndParameters="--";
};
action::action(const action& new_action):refActionDef(new_action.refActionDef){
	name=new_action.name;
	assigned_agents=new_action.assigned_agents;
	assignedParameters=new_action.assignedParameters;
	isDone=new_action.isDone;
	actionAndParameters=new_action.actionAndParameters;
	Action_GeneralParameters=new_action.Action_GeneralParameters;
	GeneralParameters=new_action.GeneralParameters;
};
action& action::operator=(const action& new_action){

	refActionDef=new_action.refActionDef;
	name=new_action.name;
	assigned_agents=new_action.assigned_agents;
	assignedParameters=new_action.assignedParameters;
	isDone=new_action.isDone;
	actionAndParameters=new_action.actionAndParameters;
	Action_GeneralParameters=new_action.Action_GeneralParameters;
	GeneralParameters=new_action.GeneralParameters;
	return *this;
};

action::~action(){};
void action::UpdateActionParamters(string assignedParametersIn, int paramterIndex){

	assignedParameters[paramterIndex]=assignedParametersIn;
	vector<string> parameterVec;
	boost::split(parameterVec, assignedParametersIn, boost::is_any_of("-"));
	if(parameterVec.size()>0)
		GeneralParameters[paramterIndex]=parameterVec[0];

	actionAndParameters=name;
	Action_GeneralParameters=name;
	for(int i=0;i<assignedParameters.size();i++)
	{
		actionAndParameters+="_"+assignedParameters[i];
		Action_GeneralParameters+="_"+assignedParameters[i];
	}

};

void action::Print(void){
	cout<<FBLU("++++++++++++ action info +++++++++++++++")<<endl;
	cout<<"Action Name: "<<name<<endl;

	cout<<"Assigned Agents: ";
	for(int i=0; i<assigned_agents.size();i++)
		cout<<assigned_agents[i]<<" ";
	cout<<endl;

	cout<<"Assigned parameters: ";
	for(int i=0; i<assignedParameters.size();i++)
		cout<<assignedParameters[i]<<" ";
	cout<<endl;

	cout<<"action with parameters: "<< actionAndParameters<<endl;

	cout<<"Is action done? ";
	for(int i=0; i<isDone.size();i++)
		cout<<isDone[i]<<" ";
	cout<<endl;

	cout<<"Assigned first part of parameters: ";
	for(int i=0; i<GeneralParameters.size();i++)
		cout<<GeneralParameters[i]<<" ";
	cout<<endl;

	cout<<"action with first part of parameters: "<< Action_GeneralParameters<<endl;

	refActionDef.Print();
	cout<<"+++++++++++++++++++++++++++++++++++++++++++"<<endl;

};

//************************************************

agent::agent(void)
{
	name="";
	type="";
	lastAssignedAction="";
	lastActionAck="";
	allowToChangePath=false;
	isBusy=false;
	isSuccessfullyDone=false;
	responsibility_number=0;
	optimal_state=0;
	next_action_index=0;
};
agent::~agent(){};
void agent::Print(void){
	cout<<FBLU("******** agent info *********")<<endl;
	cout<<"name: "<<name<<endl;
	cout<<"type: "<<type<<endl;
	cout<<"lastAssignedAction: "<<lastAssignedAction<<endl;
	cout<<"lastActionAck: "<<lastActionAck<<endl;
	cout<<"allowToChangePath: "<<allowToChangePath<<endl;
	cout<<"isBusy: "<<isBusy<<endl;
	cout<<"isSuccessfullyDone? "<<isSuccessfullyDone<<endl;
};

//****************************

offline_state_action::offline_state_action(void){
	state_name="";
};
offline_state_action::~offline_state_action(){};
void offline_state_action::Print(void){
	cout<<FBLU("+++++++++++++++++++++++ offline_state_action info +++++++++++++++++++++++")<<endl;
	cout<<"state_name: "<<state_name<<endl;

	cout<<"Actions List: "<<endl;
	for(int i=0;i<actions_list.size();i++)
		actions_list[i].Print();
};

//****************************

feasible_state_action::feasible_state_action(void){
	state_name="";
	state_type="";
	andorName="";
	state_cost=0;
	isFeasible=true;
	isSimulated=false;
};
feasible_state_action::~feasible_state_action(){};
void feasible_state_action::Print(void){
	cout<<FBLU("*********************** Feasible State-Action Table Info ***********************")<<endl;
	cout<<"The And/Or graph Name: "<<andorName<<endl;
	cout<<"state_name: "<<state_name<<endl;
	cout<<"state_type: "<<state_type<<endl;
	cout<<"state_cost: "<<state_cost<<endl;
	cout<<"isFeasible: "<<isFeasible<<endl;

	cout<<"actionsList: ";
	for(int i=0;i<actionsList.size();i++)
		cout<<actionsList[i]<<" ";
	cout<<endl;

	cout<<"actionsResponsible: ";
	for(int i=0;i<actionsResponsible.size();i++)
	{
		for(int j=0;j<actionsResponsible[i].size();j++)
		{
			cout<<actionsResponsible[i][j];
			if(actionsResponsible[i].size()>0)
				cout<<" ";
		}
		cout<<" | ";
	}
	cout<<endl;

	cout<<"actionsProgress: ";
	for(int i=0;i<actionsProgress.size();i++)
	{
		for(int j=0;j<actionsProgress[i].size();j++)
		{
			cout<<actionsProgress[i][j];
			if(actionsProgress[i].size()>0)
				cout<<" ";
		}
		cout<<" | ";
	}
	cout<<endl;

	cout<<"actions list:"<<endl;
	for(int i=0;i<actions_list.size();i++){
		actions_list[i].Print();
	}


};

//****************************
optimal_state_simulation::optimal_state_simulation(){
	state_name="";
	total_cost=0.0;
	for(int i=0;i<7;i++){
		simulation_q[0][i]=0.0;
		simulation_q[1][i]=0.0;
	}
	optimalStatePtr=NULL;
};

optimal_state_simulation::~optimal_state_simulation(){};

void optimal_state_simulation::SetAgentForAllTheAction(void){
	for(int i=0;i<actions_list.size();i++)
	{
		actions_list[i].assigned_agents=responsibleAgents;
		int NoAgents=responsibleAgents.size();
		actions_list[i].isDone.resize(NoAgents,false);
	}
};

void optimal_state_simulation::Print(void){
	cout<<FBLU("*********************** Optimal_state_simulation::Print ************************* ")<<endl;
	cout<<"state name: "<<state_name<<endl;
	cout<<"Total cost: "<<total_cost<<endl;

	cout<<"actions list: "<<endl;
	for(int i=0;i<actions_list.size();i++)
	{
		actions_list[i].Print();
		cout<<"action time: "<<actionsTime[i]<<endl;

	}
};


