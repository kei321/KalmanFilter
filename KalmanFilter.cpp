// #include "StateEq.h"
#include "ARX.h"
#include "StateSpace.h"
#include "KalmanFilter.h"
#include <math.h>
#include <random>
#include<fstream>

#define TEST_KALMAN 0
#define DEBUG_KALMAN 1

KalmanFilter::KalmanFilter(StateEq *e,double o, double p, vector<double> q):
eq(e),G(0),omega(o)
{
  buff = vector< vector<double> >(eq->A.size(),  vector<double>(eq->A.size(),0) );
  buff_p = vector< vector<double> >(eq->A.size(),  vector<double>(eq->A.size(),0) );
  P = vector< vector<double> >(eq->A.size(),  vector<double>(eq->A.size(),0) );
  Pd = vector< vector<double> >(eq->A.size(),  vector<double>(eq->A.size(),0) );
  Q = vector< vector<double> >(eq->A.size(),  vector<double>(eq->A.size(),0) );
  G.assign(eq->A.size(),p);

  for (size_t i = 0; i < P.size(); i++) {
    P[i][i] = p;
    Q[i][i] = q[i];
  }
}

double KalmanFilter::next(double output,double input)
{
  double out = 0;

  //予測ステップ
  ////事前状態推定値
  double xe = eq->next(input);
  ////事前誤差共分散行列
  for (size_t i = 0; i < P.size(); i++) {
    for (size_t j = 0; j < P.size(); j++) {
      buff[i][j] = P[i][i]*eq->A[j][i]; //Aは転地
    }
  }
  for (size_t i = 0; i < eq->B[0].size(); i++) {
    for (size_t j = 0; j < eq->B.size(); j++) {
      buff_p[i][j] = Q[i][i]*eq->B[j][i]; //Aは転地
    }
  }
  for (size_t i = 0; i < eq->A[0].size(); i++) {
    for (size_t j = 0; j < eq->A.size(); j++) {
      Pd[i][j] = 0;
      for (size_t k = 0; k < eq->A.size(); k++) {
        Pd[i][j] += eq->A[i][k]*buff[k][j];
      }
      for (size_t k = 0; k < eq->B[0].size(); k++) {
        Pd[i][j] += eq->B[i][k]*buff_p[k][j];
      }
      #if DEBUG_KALMAN
      cout << Pd[i][j] << ",";
      #endif
    }
    #if DEBUG_KALMAN
    cout << endl;
    #endif
  }
  //フィルタリングステップ
  ////カルマンゲイン
  double Pb=0;
  for (size_t i = 0; i < P.size(); i++) {
    buff[i][0]=0;
    for (size_t j = 0; j < P.size(); j++) {
      buff[i][0] += Pd[i][j]*eq->C[j];
    }
    Pb += buff[i][0]*eq->C[i];
  }
  for (size_t i = 0; i < G.size(); i++) {
    G[i] = buff[i][0]/(Pb+omega*omega);
    eq->x[i] += G[i]*(output - eq->x[i]);
  }
  out = eq->x[0];

  ////事後誤差共分散行列
  for (size_t i = 0; i < G.size(); i++) {
    for (size_t j = 0; j < eq->C.size(); j++) {
      buff[i][j] = G[i]*eq->C[j];
      if(i==j){buff[i][j] = 1-buff[i][j];}
    }
  }
  for (size_t i = 0; i < P.size(); i++) {
    for (size_t j = 0; j < P.size(); j++) {
      P[i][j] = 0;
      for (size_t k = 0; k < P.size(); k++) {
        P[i][j] += buff[i][k]*Pd[k][j];
      }
      #if DEBUG_KALMAN
      cout << P[i][j] << ",";
      #endif
    }
    #if DEBUG_KALMAN
    cout << endl;
    #endif
  }
  #if DEBUG_KALMAN
  cout << "eq->x[0]=" << eq->x[0] << ",est=" << out  << endl;
  cout << "G=" << G[0]<< ","  << G[1] << ",P11=" << P[0][0]<< endl;
  #endif

  return out;
}

//// TEST
#if TEST_KALMAN
int main(int argc, char const *argv[]) {
  ofstream ofs("Test.csv"); //ファイル出力ストリーム

  //フィルター用のモデル
  StateEq *s;
  StateSpace *ss;
  double k=0.2,m=0.5,d=0.1;
  vector< vector<double> > a = {{0,1},{-k/m,-d/m}};
  vector< vector<double> > b = {{0},{1/m}};
  vector<double> c = {1,0};
  vector<double> x = {1,0};
  ss = new StateSpace(2,0.1);
  ss->A = a; ss->B = b; ss->C = c; ss->x=x;

  s = ss;
  KalmanFilter kf(s, 10, 1000,5);

  //計測対象のモデル
  StateSpace ss_t(2,0.1);;
  k=0.2;m=0.5;d=0.1;
  vector< vector<double> > at = {{0,1},{-k/m,-d/m}};
  vector< vector<double> > bt = {{0},{1/m}};
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
  int i_max = 200;
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
    ofs << kf.G[0]<< ',' << kf.G[1]<< ',' ;
    ofs << endl;
  }

  return 0;
}
#endif
