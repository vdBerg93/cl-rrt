#ifndef TIMER_H
#define TIMER_H
#include <ctime>

struct Timer{
	clock_t tstart, tnow;
	double diff, timeLimit;
	Timer(double _timeLimit): tstart(clock()), timeLimit(_timeLimit){}
	bool Get(){
		tnow = clock();
		diff = diffclock(tnow,tstart);
		return 0 + (diff<timeLimit);
	}
	double diffclock(clock_t clock1, clock_t clock2){
		double diffticks = clock1 - clock2;
		double diffms = (diffticks)/(CLOCKS_PER_SEC/1000);
		return diffms;
	}
};

#endif