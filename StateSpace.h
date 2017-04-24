/*
StateSpace:状態方程式による離散系の伝達関数表現
*/
#ifndef STATESPACEMODEL_H_
#define STATESPACEMODEL_H_

#include <iostream>
#include <vector>
#include "Eigen/Core"
#include "Eigen/LU"

using namespace Eigen;

using namespace std;

class StateSpace
{
public:

  MatrixXf A;
  MatrixXf B;
  MatrixXf C;
  MatrixXf X;
  MatrixXf U;
  MatrixXf Y;
  double dt;
  StateSpace(MatrixXf AA, MatrixXf BB, MatrixXf CC, MatrixXf Xinit,  double deltaT);
  virtual ~StateSpace();
  double next(MatrixXf input);

private:
  MatrixXf Xn;
  // double dt;
};

#endif
