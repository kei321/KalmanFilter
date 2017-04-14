#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_

#include "StateEq.h"

class KalmanFilter
{
public:
  KalmanFilter(StateEq *e,double o, double p, vector<double> q);
  double next(double output,double input);
  void reset()
  {
    omega=0;
  }
  vector<double> G;
  StateEq *eq;
private:
  vector< vector<double> > Q;
  vector< vector<double> > P;
  vector< vector<double> > Pd;
  vector< vector<double> > buff;
  vector< vector<double> > buff_p;
  double omega;
};

#endif
