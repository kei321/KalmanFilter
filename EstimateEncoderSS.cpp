/*
カルマンフィルタのエンコーダ値推定への利用テスト
ARXモデルから推定したエンコーダーの増減モデルについて、
ARXから状態方程式に変換（実現）して、カルマンフィルターで推定を行う。
*/

#include "KalmanFilter.h"
#include "ARX.h"
#include "gnuplot.h"

#include <math.h>
#include <random>
#include <fstream>

#define GNUPLOT_ON 1


int main(int argc, char const *argv[]) {
  ofstream ofs("Test.csv"); //ファイル出力ストリーム

  //フィルター用のモデル
  StateSpace *ss;
  //バネマス系
  MatrixXf A = MatrixXf::Zero(4,4);
  MatrixXf B = MatrixXf::Zero(4,1);
  MatrixXf C = MatrixXf::Zero(1,4);
  MatrixXf X = MatrixXf::Zero(4,1);
  MatrixXf U = MatrixXf::Zero(1,1);

  A <<  0,        1,        0,        0,\
        0,        0,        1,        0,\
        0,        0,        0,        1,\
        -0.0136,   0.0145,  -0.5846,  -0.4163;

  B <<  0,  0,  0,  1;
  C <<  0.0321,   0.0063,   0.0024,  -0.0028;
  X << 0, 0,  0,  0;
  C = C*30;

  ss = new StateSpace(A,B,C,X,0.04);

  MatrixXf pp = MatrixXf::Ones(4,1);
  MatrixXf P = pp.asDiagonal()*10000;
  // MatrixXf P = MatrixXf::Zero(4,4);
  MatrixXf Q = MatrixXf::Zero(1,1);

  // P << 1000, 0, 0, 0;
  Q << 1;
  double R = 0.8;

  KalmanFilter kf(ss, P, Q, R);

  //計測対象のモデル
  ARX arx(4);
  std::vector<double> a = {0.4163,  0.5846,  -0.0145, 0.0136};
  std::vector<double> b = {-0.0028, 0.0024,  0.0063, 0.0321};
  arx.A=a;
  arx.B=b;

  double est,ot,model;
  double input,obs;
  double input_size=60;
  int i_max = 100;

  cout << "Simulation Start >>>>>>" << endl;

  for (size_t i = 0; i < i_max; i++)
  {
    // input = 10*sin(M_PI*i*0.05);
    U(0) = i > (i_max/3) ? input_size : 0;
    U(0) = i > (i_max*2/3) ? 0 : U(0);

    ot = arx.next(U(0));
    obs = ot + (-rand()+rand())*0.0002;
    est = kf.next(obs,U);
    // model = s->next(input);

    cout << est << endl;

    ofs << U(0) << ',' ;
    ofs << est << ',' << obs << ',' << ot << ',';
    ofs << kf.G(0)<< ',' << kf.G(1)<< ',' ;
    ofs << endl;
  }
#if GNUPLOT_ON
  plotCSV();
#endif
  return 0;
}
