// #include "StateEq.h"
#include "ARX.h"
#include "KalmanFilter.h"
#include <math.h>
#include <random>
#include<fstream>

#define TEST_KALMAN 1
#define DEBUG_KALMAN 1

KalmanFilter::KalmanFilter(StateEq *e,double o, double p):
eq(e),G(0),omega(o)
{
  P.assign(eq->param.size(),p);
}

double KalmanFilter::next(double output,double input)
{
  double out = 0;

  //予測ステップ
  ////事前状態推定値
  double xe = eq->next(input);
  ////事前誤差共分散行列
  double Pd=0;
  for (size_t i = 0; i < P.size(); i++) {
    Pd += eq->param[i]*P[i]*eq->param[i];
  }
  //フィルタリングステップ
  ////カルマンゲイン
  G = Pd/(Pd+omega*omega);
  out = xe + G*(output-xe);

  ////事後誤差共分散行列
  for (size_t i = 0; i < P.size()-1; i++) {
    P[i] = (1-G)*Pd;
  }
  cout << "G=" << G << ",P=" << P[0] << endl;

  return out;
}

//// TEST
#if 1
int main(int argc, char const *argv[]) {
  ofstream ofs("Test.csv"); //ファイル出力ストリーム

  //フィルター用のモデル
  StateEq *s;
  ARX *arx = new ARX(5);
  std::vector<double> a = {0.01, 0.5,  0.01, 0.1, 0.01};
  std::vector<double> b = {0.01, 0.05,  0.05, 0.05, 0.01};
  arx->setParam(a,b);
  s=arx;

  KalmanFilter kf(s, 0.01, 0.001);

  //計測対象のモデル
  ARX arx_t(5);
  std::vector<double> at = {0.03, 0.2,  0.01, 0.15, 0.015};
  std::vector<double> bt = {0.05, 0.02,  0.02, 0.01, 0.01};
  arx_t.setParam(at,bt);

  double est,ot,model;
  double input,obs;
  for (size_t i = 0; i < 300; i++)
  {
    input = 10*sin(M_PI*i*0.05);
    ot = arx_t.next(input);
    obs = ot + rand()*0.0001;
    est = kf.next(obs,input);
    model = arx->next(input);

    cout << est << endl;
    ofs << est << ',' << obs << ',' << ot << ',' << input;
    ofs << ',' << model;
    ofs << endl;
  }

  return 0;
}
#endif
