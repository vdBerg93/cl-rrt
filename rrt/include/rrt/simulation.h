#ifndef SIMULATION_H
#define SIMULATION_H

#include "rrt/datatypes.h"
#include "rrt/controller.h"

class Simulation{
	private:
		void propagate(const MyRRT& RRT, Controller control, const MyReference& ref, const Vehicle& veh);
	public:
		StateArray stateArray;
		vector<double> curvature;
		vector<int> closestPoints;
		vector<double> acmd,dcmd; // controls logging
		double costS, costE;
		
		bool goalReached, endReached;
		Simulation(	const MyRRT& RRT, const vector<double>& state, MyReference& ref, const Vehicle& veh, 
				   	const bool& GoalBiased, const bool& genProfile, const double& Vstart);
		bool isvalid();
};
bool Simulation::isvalid()
{
	return endReached;
}

#endif