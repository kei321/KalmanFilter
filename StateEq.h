#ifndef STATEEQ_H_
#define STATEEQ_H_

#include <iostream>
#include <vector>

using namespace std;

class StateEq
{
public:
  vector< vector<double> > A;
  vector<double> B;
  vector<double> C;

  StateEq(){};
  virtual double next(double input){};

  void shift(std::vector<double> &v, double insert)
  {
      for(int i=v.size()-1;i>0;i--)
      {
        v[i]=v[i-1];
      }
      v[0]=insert;
  }
};

#endif
