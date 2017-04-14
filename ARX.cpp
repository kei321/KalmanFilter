#include "ARX.h"

#define DEBUG 1

ARX::ARX(int size)
{
  u.assign(size,0);
  y.assign(size,0);
  // a.assign(size,0);
  // b.assign(size,0);
  A.assign(size,0);
  B.assign(size,0);
  C.assign(size,0);
}

ARX::~ARX(){
}

double ARX::next(double input)
{
  double output=0;
  for(int i=0;i<u.size();i++)
  {
    output += A[i]*y[i] + B[i]*u[i];
    #if DEBUG
    cout << y[i] << ",";
    #endif
  }
  #if DEBUG
  cout << endl;
  for(int i=0;i<u.size();i++)
  {
    cout << u[i] << ",";
  }
  cout << endl;
  #endif

  shift(u,input);
  shift(y,output);

  return output;
}

/*
  test code
*/
#if 1
#include<fstream>
int main(int argc, char const *argv[]) {
  ofstream ofs("Test.csv"); //ファイル出力ストリーム

  ARX arx(4);
  std::vector<double> a = {0.4163,  0.5846,  -0.0145, 0.0136};
  std::vector<double> b = {-0.0028, 0.0024,  0.0063, 0.0321};
  arx.A=a;
  arx.B=b;

  double o,input;
  int i_max = 100;
  for (size_t i = 0; i < i_max; i++)
  {

    input = i < i_max/2 ? 50 : 0;
    o = arx.next(input);
    cout << "i=" << i << ": ";
    cout << "output= " << o << endl;

    ofs << o << ',' << input;
    ofs << endl;
  }

  return 0;
}
#endif
