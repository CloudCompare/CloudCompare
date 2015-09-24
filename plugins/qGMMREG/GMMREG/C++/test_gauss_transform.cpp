#include <assert.h>

#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>

#include "gmmreg_utils.h"

using namespace std;

int main(int argc, char* argv[]) {
  if (argc < 4) {
    cerr << "Usage: "<< argv[0]<<" pts1 pts2 scale [gradient]" << endl;
    cerr << "  pts1, pts2, gradient(optional) -- ascii text files " << endl;
    cerr << "  scale -- numerical scalar value" << endl;
    cerr << "Example: " << argv[0] << " pts1.txt pts2.txt 1.0 gradient.txt"
         << endl;
    return -1;
  }
  vnl_matrix<double> A;
  vnl_matrix<double> B;

  ifstream file1(argv[1]);
  A.read_ascii(file1);

  ifstream file2(argv[2]);
  B.read_ascii(file2);

  int m = A.rows();
  int n = B.rows();
  int d = A.cols();
  assert(m>0 && n>0 && d>0 && B.cols()==d);

  vnl_matrix<double> gradient;
  gradient.set_size(m,d);
  gradient.fill(0);
  double scale = atof(argv[3]);
  double cost = 0;
  clock_t start, end;
  double elapsed;

  start = clock();

  cost = gmmreg::GaussTransform(A, B, scale, gradient);
  end = clock();
  elapsed = 1000*((double) (end - start)) / CLOCKS_PER_SEC;
  cout << "Evaluate Gauss Transform: " << cost
       << " in " <<  elapsed << " ms." << endl;
  if (argc > 4) {
    ofstream outfile(argv[4],ios_base::out);
    gradient.print(outfile);
  }
  // system("pause");
  return 0;
}
