/*
StateSpace:状態方程式による離散系の伝達関数表現
*/
#ifndef STATESPACEMODEL_H_
#define STATESPACEMODEL_H_

#include <iostream>
#include <vector>
#include "StateEq.h"

using namespace std;

class StateSpace : public StateEq
{
public:
  vector<double> u;
  vector<double> x;
  vector<double> xb;  //next step
  vector<double> y;

  StateSpace(int size, double deltaT);
  virtual ~StateSpace();
  double next(double input);
  // void setParam(std::vector<double> aa, std::vector<double> bb)
  // {
  //   a=aa;
  //   b=bb;
  //   // param[0]=a[0];
  //   for (size_t i = 0; i < param.size(); i++) {
  //     if(i<aa.size()){param[i]=aa[i];}
  //     else{param[i]=bb[i];}
  //   }
  //
  //   // cout << "param=" << param[0] << endl;
  // }

private:
  double dt;
};

#endif
