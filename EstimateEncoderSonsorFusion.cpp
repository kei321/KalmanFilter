/*
カルマンフィルタのエンコーダ値推定への利用テスト
ARXモデルから推定したエンコーダーの増減モデルについて、
カルマンフィルターを利用した相補フィルターで推定を行う。
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
  double aa=0.85;
  MatrixXf A = MatrixXf::Zero(3,3);
  MatrixXf B = MatrixXf::Zero(3,2);
  MatrixXf C = MatrixXf::Zero(1,3);
  MatrixXf X = MatrixXf::Zero(3,1);
  MatrixXf E = MatrixXf::Zero(2,1);
  A <<  0.1,    0,    0,  \
        0,    1,    0,  \
        0,    0,    0;
  B <<  1,    0,  \
        0,    0,  \
        0,    1;
  C <<  1,    1,    -1;
  X <<  0,    0.05,    2;
  ss = new StateSpace(A,B,C,X,0.1);
  ss->switch2NotRealTime = true;

  MatrixXf P = MatrixXf::Zero(3,3);
  MatrixXf Q = MatrixXf::Zero(2,2);

  P <<  100000,    0,    0,  \
        0,    100000,    0,  \
        0,    0,    10000;
  Q <<  1,    0,  \
        0,    1;
  double R = 0.7;

  KalmanFilter kf(ss, P, Q, R);

  //計測対象のモデル
  ARX arx(4);
  std::vector<double> a = {0.4163,  0.5846,  -0.0145, 0.0136};
  std::vector<double> b = {-0.0028, 0.0024,  0.0063, 0.0321};
  arx.A=a;
  arx.B=b;

  //計測対象を推定したモデル
  double ee=1.2;//推定誤差
  ARX arx_es(4);
  std::vector<double> aes = {0.4163,  0.5846,  -0.0145, 0.0136};
  std::vector<double> bes = {-0.0028, 0.0024,  0.0063, 0.0321*ee};
  arx_es.A=aes;
  arx_es.B=bes;

  double est,ot,model;
  double input_size=60;
  double input,obs1,obs2,mu=ss->X(0),beta=ss->X(1);
  double e1,e2,e=0;
  int i_max = 100;
  MatrixXf U = MatrixXf::Zero(1,1);

  cout << "Simulation Start >>>>>>" << endl;

  for (size_t i = 0; i < i_max; i++)
  {
    // input = 10*sin(M_PI*i*0.05);
    U(0) = i > (i_max/8) ? input_size : 0;
    U(0) = i > (i_max*2/8) ? 0 : U(0);

    ot = arx.next(U(0));
    e += (i%2==0) ? (-rand()*0.8+rand())*input_size*0.0000001 : 0;
    obs1 = ot + e + (-rand()*0.8+rand())*0.00004;
    obs2 = arx_es.next(U(0));
    kf.next(obs2 - obs1, E);
    //
    e1 = kf.eq->X(0) + kf.eq->X(1);
    e2 = kf.eq->X(2);

    est = obs1 - e2;

    cout << est << endl;

    cout << est  << ", u=" << U(0) << ",ot=" << obs2 - obs1 <<  endl;
    ofs << U(0) << ',' ;
    ofs << est << ',' << obs1 << ',' << ot << ',' << obs2 << ',' ;
    ofs << kf.G(0)<< ',' << kf.G(1)<< ',' ;
    ofs << endl;
  }
#if GNUPLOT_ON
  plotCSV();
#endif
  return 0;
}
