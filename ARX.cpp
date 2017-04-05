#include "ARX.h"

#define DEBUG 0

ARX::ARX(int size) : StateEq()
{
  u.assign(size,0);
  y.assign(size,0);
  a.assign(size,0);
  b.assign(size,0);

  param.assign(size*2,0);
}

ARX::~ARX(){
}

double ARX::next(double input)
{
  double output=0;
  for(int i=0;i<u.size()-1;i++)
  {
    output += -a[i]*y[i] + b[i]*u[i];
    #if DEBUG
    cout << y[i] << ",";
    #endif
  }

  for(int i=u.size()-1;i>0;i--)
  {
    y[i]=y[i-1];
    u[i]=u[i-1];
  }
  y[0]=output;
  u[0]=input;

  #if DEBUG
  cout << endl;
  #endif
  return output;
}

/*
  test code
*/
#if 0
int main(int argc, char const *argv[]) {
  StateEq *s;
  ARX arx(5);
  std::vector<double> a = {1.0, 0.5,  0.01, 0.001, 0.0001};
  std::vector<double> b = {2.0, 0.05,  0.05, 0.005, 0.0001};
  arx.a=a;
  arx.b=b;
  s=&arx;

  double o;
  for (size_t i = 0; i < 10; i++)
  {
    o = s->next(0.1);
    cout << "output=" << o << endl;
  }

  return 0;
}
#endif
