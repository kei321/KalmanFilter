#ifndef STATEEQ_H_
#define STATEEQ_H_

#include <iostream>
#include <vector>

class StateEq
{
public:
  StateEq(){};
  virtual double next(double input){};
  void test(){};

  std::vector<double> param;
};

#endif
