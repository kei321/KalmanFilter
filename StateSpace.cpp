/*
StateSpace:状態方程式による離散系の伝達関数表現
*/
#include "StateSpace.h"

#define DEBUG 0
#define TEST_SS 0

StateSpace::StateSpace(MatrixXf AA, MatrixXf BB, MatrixXf CC, MatrixXf Xinit, double deltaT)
{
  A = AA;
  B = BB;
  C = CC;
  X = Xinit;
  Xn = MatrixXf::Zero(X.rows(),1);
  Y = MatrixXf::Zero(C.rows(),1);

#if DEBUG
  cout << "X:rows" << X.rows() << ",cols" << X.cols() << endl;
  cout << "Y:rows" << Y.rows() << ",cols" << Y.cols() << endl;
  cout << "A:rows" << A.rows() << ",cols"  << A.cols() << endl;
  cout << "B:rows" << B.rows() << ",cols"  << B.cols() << endl;
#endif

  dt = deltaT;
}

StateSpace::~StateSpace(){
}

double StateSpace::next(MatrixXf input)
{
  double output=0;
  //
  Xn = A*X + B*input;
  X = X + Xn * dt;  //オイラー法
  Y = C*X;
  output = Y(0);
#if DEBUG
  cout << "Y:rows" << Y.rows() << ",cols" << Y.cols() << endl;
  cout << "Y:" << Y(0)<< endl;
#endif
  return output;
}

#if TEST_SS
#include<fstream>
int main(int argc, char const *argv[]) {
  ofstream ofs("Test.csv"); //ファイル出力ストリーム
  //バネマス系
  double k=0.2,m=0.5,d=0.1;
  MatrixXf A = MatrixXf::Zero(2,2);
  MatrixXf B = MatrixXf::Zero(2,1);
  MatrixXf C = MatrixXf::Zero(1,2);
  MatrixXf X = MatrixXf::Zero(2,1);
  A <<  0,    1,  \
        -k/m, -d/m;
  B <<  0,
        1/m;
  C <<  1,    0;
  X <<  0.1,  0;

  StateSpace ss(A,B,C,X,0.1);

  double output=0,input=0;
  for (size_t i = 0; i < 100; i++) {
    input = i > 50 ? 0.1 : 0;
    output = ss.next(input);
    //
    cout << output << endl;
    ofs << output << ',' << input;
    ofs << endl;
  }
  return 0;
}
#endif
