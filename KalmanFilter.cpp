// #include "StateEq.h"
#include "ARX.h"
#include "KalmanFilter.h"
#include <math.h>

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
  for (size_t i = 0; i < P.size()-1; i++) {
    Pd += P[i]*eq->param[i]*eq->param[i];
  }
  //フィルタリングステップ
  ////カルマンゲイン
  G = Pd/(Pd+omega*omega);
  out = xe + G*(output-xe);

  ////事後誤差共分散行列
  for (size_t i = 0; i < P.size()-1; i++) {
    P[i] = (1-G)*Pd;
  }
  cout << "G=" << G << endl;

  return out;
}

//// TEST
#if 1
int main(int argc, char const *argv[]) {
  StateEq *s;
  ARX arx(5);
  std::vector<double> a = {1.0, 0.5,  0.01, 0.001, 0.0001};
  std::vector<double> b = {2.0, 0.05,  0.05, 0.005, 0.0001};
  arx.setParam(a,b);
  s=&arx;

  KalmanFilter kf(s, 0.01, 0.001);

  double o;
  double input,obs;
  for (size_t i = 0; i < 100; i++)
  {
    input = sin(M_PI*0.1*i*0.01);
    obs = cos(input);
    o = kf.next(input,obs);
    cout << o << endl;
  }

  return 0;
}
#endif
