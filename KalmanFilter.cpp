// #include "StateEq.h"
#include "KalmanFilter.h"
#include <math.h>
#include <random>
#include<fstream>

#define TEST_KALMAN 1
#define DEBUG_KALMAN 1

KalmanFilter::KalmanFilter(StateSpace *e, MatrixXf P_, MatrixXf Q_, double R_)
{
  P = P_;
  Pm = P_;
  Q = Q_;
  R = R_;
  eq = e;

#if DEBUG_KALMAN
  cout << "P:rows" << P.rows() << ",cols" << P.cols() << endl;
  cout << "Q:rows" << Q.rows() << ",cols"  << Q.cols() << endl;
#endif

}

double KalmanFilter::next(double output,double input)
{
  double out = 0;

  ////事前状態推定値
  eq->next(input);

  Pm = eq->A*P*eq->A.transpose() + eq->B*Q*eq->B.transpose();
  // G = (eq->C*Pm*eq->C.transpose());
  G = Pm*eq->C.transpose();
  cout << "Buff" << Buff.size() << "G" << G.size() << endl;
  // G = G.inverse()*Buff;
  eq->Y(0,0) = output;
  Xnew = eq->X + G*( eq->Y - eq->C*eq->X );
  out = Xnew(0,0);

  cout << out << endl;

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
  A <<  0,    1,  \
        -k/m, -d/m;
  B <<  0,
        1/m;
  C <<  1,    0;
  X <<  0.1,  0;

  ss = new StateSpace(A,B,C,X,0.1);

  MatrixXf P = MatrixXf::Zero(2,2);
  MatrixXf Q = MatrixXf::Zero(1,1);

  P << 100, 0, 0, 100;
  Q << 0.01;
  double R = 0.01;

  KalmanFilter kf(ss, P, Q, R);

  //計測対象のモデル
  A <<  0,    1,  \
        -k/m, -d/m;
  B <<  0,
        1/m;
  C <<  1,    0;
  X <<  0.1,  0;

  StateSpace ss_t(A,B,C,X,0.1);

  double est,ot,model;
  double input,obs;
  int i_max = 20;
  for (size_t i = 0; i < i_max; i++)
  {
    // input = 10*sin(M_PI*i*0.05);
    input = i > (i_max/2) ? 1 : 0;
    input = i > (i_max*2/3) ? 0 : input;

    ot = ss_t.next(input);
    obs = ot + (-0.5 + rand())*0.00005;
    est = kf.next(obs,input);
    // model = s->next(input);

    cout << est << endl;
    ofs << input << ',' ;
    ofs << est << ',' << obs << ',' << ot << ',';
    // ofs << kf.G[0]<< ',' << kf.G[1]<< ',' ;
    ofs << endl;
  }

  return 0;
}
#endif
