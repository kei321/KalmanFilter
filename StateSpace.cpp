/*
StateSpace:状態方程式による離散系の伝達関数表現
*/
#include "StateSpace.h"

#define DEBUG 0

StateSpace::StateSpace(int size, double deltaT) : StateEq()
{
  u.assign(size,0);
  x.assign(size,0);
  xb.assign(size,0);
  y.assign(size,0);
  A = vector< vector<double> >(size,  vector<double>(size,0) );
  B.assign(size,0);
  C.assign(size,0);

  // int param_size = A.size()
  // param.assign(param_size,0);

  dt = deltaT;
}

StateSpace::~StateSpace(){
}

double StateSpace::next(double input)
{
  double output=0;
  for (size_t i = 0; i < A.size(); i++) {
    for (size_t j = 0; j < A[0].size(); j++) {
      xb[i] += A[i][j]*x[j] + B[j]*input;
    }
    xb[i] = x[i] + xb[i]*dt;
    #if DEBUG
    cout << xb[i] << ",";
    #endif
  }
  x = xb;
  for (size_t i = 0; i < C.size(); i++) {
    output += C[i]*x[i];
    xb[i] = 0;
  }
  return output;
}

#if 0
#include<fstream>
int main(int argc, char const *argv[]) {
  ofstream ofs("Test.csv"); //ファイル出力ストリーム
  //バネマス系
  double k=0.2,m=0.5,d=0.1;
  vector< vector<double> > a = {{0,1},{-k/m,-d/m}};
  vector<double> b = {0,1/m};
  vector<double> c = {1,0};
  vector<double> x = {1,0};

  StateSpace ss(2,0.1);
  ss.A = a;ss.B = b;ss.C = c;ss.x=x;

  double output=ss.x[1],input=0;
  for (size_t i = 0; i < 100; i++) {
    // input = i < 50 ? 1 : 0;
    output = ss.next(input);

    cout << endl;
    ofs << output << ',' << input;
    ofs << endl;
  }
  return 0;
}
#endif
