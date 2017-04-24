// #include "StateEq.h"
#include "KalmanFilter.h"
// #include "gnuplot.h"

#include <math.h>
#include <random>
#include <fstream>
#include <cstdio>
#include <cstdlib>

#define TEST_KALMAN 0
#define DEBUG_KALMAN 0
#define WINDOWS 1

KalmanFilter::KalmanFilter(StateSpace *e, MatrixXf P_, MatrixXf Q_, double R_)
{
  P = P_;
  Pm = P_;
  Pb = P_;
  Q = Q_;
  R = R_;
  eq = e;
  G = MatrixXf::Zero(eq->C.rows(),eq->C.cols());
  // Buff = MatrixXf::Zero(eq->C.rows(),eq->C.cols());

#if DEBUG_KALMAN
  cout << "P:rows" << P.rows() << ",cols" << P.cols() << endl;
  cout << "Q:rows" << Q.rows() << ",cols"  << Q.cols() << endl;
#endif

}

double KalmanFilter::next(double output, MatrixXf input)
{
  double out = 0,buff=1;

  ////事前状態推定値
  eq->next(input);

  Pm = eq->A*P*eq->A.transpose() + eq->B*Q*eq->B.transpose();
  // cout << Pm.size() << endl;
  Buff = ( eq->C*Pm*eq->C.transpose() );
#if DEBUG_KALMAN
  cout << "Buff:" << Buff(0) << endl;//<< "," << Buff(1) << endl;
#endif
  buff = Buff(0) + R*R;
  Buff = Pm*eq->C.transpose() ;
  G = Buff / buff;
  // cout << "Buff" << Buff.size() << "G" << G.size() << endl;
  Xnew = eq->X + G*output - G*eq->C*eq->X ;
  eq->X = Xnew;
  for (size_t i = 0; i < G.rows(); i++) {
    for (size_t j = 0; j < eq->C.cols(); j++) {
      double b= (i==j) ? 1 : 0;
      Pb(i,j) = b - eq->C(j) * G(i);
      #if DEBUG_KALMAN
      cout << Pb(i,j) << "," << endl;
      #endif
    }
  }
  P = Pb*Pm;
#if DEBUG_KALMAN
  cout << "G:" << G(0) << "," << G(1) << endl;
  cout << "P:" << P(0) << "," << P(1) << endl;
  cout << "Xnew:" << Xnew.size() << "Y" << eq->Y.rows() << endl;
  cout << "Xnew:" << Xnew(0) << "," << Xnew(1) << endl;
#endif
  out = Xnew(0);

  // cout << out << endl;

  return out;
}

//// TEST
#if TEST_KALMAN
int main(int argc, char const *argv[]) {
  ofstream ofs("Test.csv"); //ファイル出力ストリーム

  //フィルター用のモデル
  StateSpace *ss;
  //バネマス系
  double k=0.2,m=0.5,d=0.1;
  MatrixXf A = MatrixXf::Zero(2,2);
  MatrixXf B = MatrixXf::Zero(2,1);
  MatrixXf C = MatrixXf::Zero(1,2);
  MatrixXf X = MatrixXf::Zero(2,1);
  MatrixXf U = MatrixXf::Zero(1,1);
  A <<  0,    1,  \
        -k/m, -d/m;
  B <<  0,
        1/m;
  C <<  1,    0;
  X <<  0.1,  0;

  ss = new StateSpace(A,B,C,X,0.1);

  MatrixXf P = MatrixXf::Zero(2,2);
  MatrixXf Q = MatrixXf::Zero(1,1);

  P << 1000, 0, 0, 100;
  Q << 0.05;
  double R = 0.8;

  KalmanFilter kf(ss, P, Q, R);

  //計測対象のモデル
  k=k+0.01;m=m+0.01;d=d+0.05;
  A <<  0,    1,  \
        -k/m, -d/m;
  B <<  0,
        1/m;
  C <<  1,    0;
  X <<  0.1,  0;

  StateSpace ss_t(A,B,C,X,0.1);

  double est,ot,model;
  double input,obs;
  int i_max = 200;
  for (size_t i = 0; i < i_max; i++)
  {
    // input = 10*sin(M_PI*i*0.05);
    U(0) = i > (i_max/3) ? 1 : 0;
    U(0) = i > (i_max*2/3) ? 0 : U(0);

    ot = ss_t.next(U);
    obs = ot + (-rand()+rand())*0.00005;
    est = kf.next(obs,U);
    // model = s->next(input);

    cout << est << endl;

    ofs << U(0) << ',' ;
    ofs << est << ',' << obs << ',' << ot << ',';
    ofs << kf.G(0)<< ',' << kf.G(1)<< ',' ;
    ofs << endl;
  }
  plotCSV();

  return 0;
}
#endif
