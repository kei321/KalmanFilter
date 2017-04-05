#ifndef ARXMODEL_H_
#define ARXMODEL_H_

#include <iostream>
#include <vector>
#include "StateEq.h"
using namespace std;

class ARX : public StateEq
{
public:
  std::vector<double> u;
  std::vector<double> y;
  std::vector<double> a;
  std::vector<double> b;

  ARX(int size);
  virtual ~ARX();
  double next(double input);
  void setParam(std::vector<double> aa, std::vector<double> bb)
  {
    a=aa;
    b=bb;
    for (size_t i = 0; i < param.size()-1; i++) {
      if(i<aa.size()){param[i]=aa[i];}
      else{param[i]=bb[i];}
    }
  }

private:

};

#endif
