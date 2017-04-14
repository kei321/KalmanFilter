/*
ARX:差分方程式による離散系の伝達関数表現
*/

#ifndef ARXMODEL_H_
#define ARXMODEL_H_

#include <iostream>
#include <vector>
using namespace std;

class ARX
{
public:
  vector<double> A;
  vector<double> B;
  vector<double> C;
  vector<double> x;
  std::vector<double> u;
  std::vector<double> y;
  std::vector<double> a;
  std::vector<double> b;

  ARX(int size);
  virtual ~ARX();
  double next(double input);
  void shift(std::vector<double> &v, double insert)
  {
      for(int i=v.size()-1;i>0;i--)
      {
        v[i]=v[i-1];
      }
      v[0]=insert;
  }

private:

};

#endif
