/*
相補フィルタのテスト
*/

#include "KalmanFilter.h"
#include "gnuplot.h"

#include <math.h>
#include <random>
#include <fstream>

int main(int argc, char const *argv[]) {
  ofstream ofs("Test.csv"); //ファイル出力ストリーム

  //フィルター用のモデル
  StateSpace *ss;
  double aa=0.75;
  MatrixXf A = MatrixXf::Zero(3,3);
  MatrixXf B = MatrixXf::Zero(3,2);
  MatrixXf C = MatrixXf::Zero(1,3);
  MatrixXf X = MatrixXf::Zero(3,1);
  MatrixXf E = MatrixXf::Zero(2,1);
  A <<  aa,    0,    0,  \
        0,    1,    0,  \
        0,    0,    0;
  B <<  1-aa,    0,  \
        0,    0,  \
        0,    1;
  C <<  1,    1,    -1;
  X <<  10,    30,    2;
  ss = new StateSpace(A,B,C,X,0.1);

  MatrixXf P = MatrixXf::Zero(3,3);
  MatrixXf Q = MatrixXf::Zero(2,2);

  P <<  1000,    0,    0,  \
        0,    1000,    0,  \
        0,    0,    1000;
  Q <<  0.001,    0,  \
        0,        0.001;
  double R = 0.001;

  KalmanFilter kf(ss, P, Q, R);

  //計測対象のモデル
  //バネマス系
  double k=0.2,m=0.5,d=0.1;
  MatrixXf AA = MatrixXf::Zero(2,2);
  MatrixXf BB = MatrixXf::Zero(2,1);
  MatrixXf CC = MatrixXf::Zero(1,2);
  MatrixXf XX = MatrixXf::Zero(2,1);
  MatrixXf U = MatrixXf::Zero(1,1);
  AA <<  0,    1,  \
        -k/m, -d/m;
  BB <<  0,
        1/m;
  CC <<  1,    0;
  XX <<  0.1,  0;

  StateSpace ss_t(AA,BB,CC,XX,0.1);

  double est,ot,model;
  double input,obs1,obs2,mu=ss->X(0);
  double e1,e2;
  int i_max = 200;
  for (size_t i = 0; i < i_max; i++)
  {
    // input = 10*sin(M_PI*i*0.05);
    U(0) = i > (i_max/3) ? 1 : 0;
    U(0) = i > (i_max*2/3) ? 0 : U(0);

    ot = ss_t.next(U);
    obs1 = ot + (-rand()+rand())*0.00008;
    mu = aa*mu + (1-aa)*(-rand()+rand())*0.0002;
    obs2 = ot + mu;
    kf.next(obs2 - obs1, E);
    //
    e1 = kf.eq->X(0) + kf.eq->X(1);
    e2 = kf.eq->X(2);

    est = e1;

    cout << est  << ". u=" << U(0) << endl;
    ofs << U(0) << ',' ;
    ofs << est << ',' << obs2 << ',' << ot << ',';
    ofs << kf.G(0)<< ',' << kf.G(1)<< ',' ;
    ofs << endl;
  }
  plotCSV();

  return 0;
}
