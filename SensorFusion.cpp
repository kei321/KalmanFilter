/*
相補フィルタのテスト
*/

#include "KalmanFilter.h"
#include "StateSpace.h"

#include <math.h>
#include <random>
#include<fstream>

int main(int argc, char const *argv[]) {
  ofstream ofs("Test.csv"); //ファイル出力ストリーム

  //フィルター用のモデル
  StateEq *s;
  StateSpace *ss;
  double aa=0.75;
  vector< vector<double> > a = {{1,0,0},{0,1,0},{0,0,0}};
  vector< vector<double> > b = {{1,0},{0,0},{0,1}};
  vector<double> c = {1,1,-1};
  vector<double> x = {0,1,2};
  vector<double> Q = {5,2000};
  ss = new StateSpace(3,0.1);
  ss->A = a; ss->B = b; ss->C = c; ss->x=x;

  s = ss;
  KalmanFilter kf(s, 0.0000000001, 10000000, Q);

  //計測対象のモデル
  StateSpace ss_t(2,0.1);;
  double k=0.2,m=0.5,d=0.1;
  vector< vector<double> > at = {{0,1},{-k/m,-d/m}};
  vector< vector<double> > bt = {{0},{1/m}};
  vector<double> ct = {1,0};
  vector<double> xt = {1,0};
  ss_t.A = at; ss_t.B = bt; ss_t.C = ct; ss_t.x=xt;

  double est,ot,model;
  double input,obs;
  double e1,e2;
  int i_max = 10;
  for (size_t i = 0; i < i_max; i++)
  {
    // input = 10*sin(M_PI*i*0.05);
    input = i > (i_max/2) ? 1 : 0;
    input = i > (i_max*2/3) ? 0 : input;

    ot = ss_t.next(input);
    obs = ot + (-0.5 + rand())*0.00005;
    kf.next(obs - ot,0);

    e1 = kf.eq->x[0] + kf.eq->x[1];
    e2 = kf.eq->x[2];

    est = e1;

    cout << est << endl;
    ofs << input << ',' ;
    ofs << est << ',' << obs << ',' << ot << ',';
    ofs << kf.G[0]<< ',' << kf.G[1]<< ',' ;
    ofs << endl;
  }

  return 0;
}
