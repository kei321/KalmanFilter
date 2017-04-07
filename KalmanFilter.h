#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#include "StateEq.h"

class KalmanFilter
{
public:
  KalmanFilter(StateEq *e,double o, double p);
  double next(double output,double input);
  void reset()
  {
    omega=0;
  }
  vector<double> G;
private:
  vector< vector<double> > P;
  vector< vector<double> > Pd;
  vector< vector<double> > buff;
  vector< vector<double> > buff_p;
  double omega;
  StateEq *eq;
};

#endif
