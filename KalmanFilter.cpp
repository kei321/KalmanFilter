// #include "StateEq.h"
#include "ARX.h"
#include "StateSpace.h"
#include "KalmanFilter.h"
#include <math.h>
#include <random>
#include<fstream>

#define TEST_KALMAN 1
#define DEBUG_KALMAN 1

KalmanFilter::KalmanFilter(StateEq *e,double o, double p):
eq(e),G(0),omega(o)
{
  buff.assign(eq->A.size(),0);
  P.assign(eq->A.size(),p);
  Pd.assign(eq->A.size(),0);
}

double KalmanFilter::next(double output,double input)
{
  double out = 0;

  //予測ステップ
  ////事前状態推定値
  double xe = eq->next(input);
  ////事前誤差共分散行列
  for (size_t i = 0; i < P.size(); i++) {
    buff[i] = P[i]*eq->A[0][i];
    cout << "buff=" << buff[i] << endl;
  }
  for (size_t i = 0; i < eq->A[0].size(); i++) {
    for (size_t j = 0; j < eq->A.size(); j++) {
      Pd[i] += eq->A[i][j]*buff[j];
    }
    cout << "Pd=" << Pd[i] << endl;
  }
  //フィルタリングステップ
  ////カルマンゲイン
  double Pb=0;
  for (size_t i = 0; i < P.size(); i++) {
    Pb += P[i]*eq->C[i];
  }
  G = Pb/(Pb+omega*omega);
  out = xe + G*(output-xe);

  ////事後誤差共分散行列
  for (size_t i = 0; i < P.size(); i++) {
    P[i] = (1-G)*Pd[i];
  }
  cout << "G=" << G << endl;

  return out;
}

//// TEST
#if 1
int main(int argc, char const *argv[]) {
  ofstream ofs("Test.csv"); //ファイル出力ストリーム

  //フィルター用のモデル
  StateEq *s;
  StateSpace *ss;
  double k=0.2,m=0.5,d=0.1;
  vector< vector<double> > a = {{0,1},{-k/m,-d/m}};
  vector<double> b = {0,1/m};
  vector<double> c = {1,0};
  vector<double> x = {1,0};
  ss = new StateSpace(2,0.1);
  ss->A = a; ss->B = b; ss->C = c; ss->x=x;

  s = ss;
  KalmanFilter kf(s, 0.001, 2000);

  //計測対象のモデル
  StateSpace ss_t(2,0.1);;
  k=0.2;m=0.55;d=0.1;
  vector< vector<double> > at = {{0,1},{-k/m,-d/m}};
  vector<double> bt = {0,1/m};
  vector<double> ct = {1,0};
  vector<double> xt = {1,0};
  ss_t.A = at; ss_t.B = bt; ss_t.C = ct; ss_t.x=xt;

/** ARX モデルによる検証
  //フィルター用のモデル
  StateEq *s;
  ARX *arx = new ARX(4);
  std::vector<double> a = {0.4163,  0.5846,  -0.0145, 0.0136};
  std::vector<double> b = {-0.0028, 0.0024,  0.0063,  0.0321};
  arx->setParam(a,b);
  s=arx;

  KalmanFilter kf(s, 0.01, 0.000001);

  //計測対象のモデル
  ARX arx_t(5);
  std::vector<double> at = {0.4163,  0.5846,  -0.0145, 0.0136};
  std::vector<double> bt = {-0.0028, 0.0024,  0.0063, 0.0321};
  arx_t.setParam(at,bt);
*/
  double est,ot,model;
  double input,obs;
  int i_max = 500;
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
    ofs << model<< ',' ;
    ofs << endl;
  }

  return 0;
}
#endif