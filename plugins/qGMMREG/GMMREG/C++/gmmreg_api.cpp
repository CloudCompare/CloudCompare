#include "gmmreg_api.h"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>

#ifdef WIN32
#include <windows.h>
#else
#include "port_ini.h"
#endif

#include "gmmreg_cpd.h"
#include "gmmreg_grbf.h"
#include "gmmreg_rigid.h"
#include "gmmreg_tps.h"
#include "gmmreg_utils.h"
#include "utils/misc_utils.h"

using std::cerr;
using std::cout;
using std::endl;

#ifdef __cplusplus
extern "C"
#endif

void print_usage() {
  cerr << "C++ implementation of the generic point set registration framework "
          "proposed in Bing Jian and Baba C. Vemuri, "
          "Robust Point Set Registration Using Gaussian Mixture Models, "
          "IEEE Trans. Pattern Anal. Mach. Intell., 2011(33), 1633-1645."
       << endl;
  cerr << "The following methods are currently available as special cases "
          "of the proposed framework: "
       << endl;
  cerr << " 'EM_TPS': "
          "Haili Chui and Anand Rangarajan, "
          "A new point matching algorithm for non-rigid registration, "
          "Computer Vision and Image Understanding, 2003, 89(2-3), pp. 114-141."
       << endl;
  cerr << " 'EM_GRBF': "
          "Andriy Myronenko, Xubo B. Song, Miguel A. Carreira-Perpinan, "
          "Non-rigid Point Set Registration: Coherent Point Drift,"
          "NIPS 2006, pp. 1009-1016."
       << endl;
  cerr << " 'TPS_L2, GRBF_L2': "
          "Bing Jian and Baba C. Vemuri, "
          "A Robust Algorithm for Point Set Registration Using Mixture of Gaussians, "
          "ICCV 2005, pp. 1246-1251."
       << endl;
  cerr << " 'TPS_KC, GRBF_KC': "
          "Yanghai Tsin and Takeo Kanade, "
          "A Correlation-Based Approach to Robust Point Set Registration, "
          "ECCV (3) 2004: 558-569. "
       << endl;
  cerr << " 'rigid':  rigid registration using Jian and Vemuri's algorithm."
       << endl;
}

#ifdef __cplusplus
extern "C"
#endif
int gmmreg_api(const char* input_config, const char* method) {
  cout << "Robust Point Set Registration Using Gaussian Mixture Models" << endl;
  cout << "Compiled on " << __DATE__ << ", " << __TIME__ << endl;
  cout << "Copyright Bing Jian & Baba C. Vemuri " << endl;
  char f_config[1024];
  gmmreg::utils::get_config_fullpath(input_config, f_config);

  if (!strcmp(strupr((char*)method), "EM_TPS")) {
    gmmreg::CoherentPointDriftTps().Run(f_config);
  } else if (!strcmp(strupr((char*)method), "EM_GRBF")) {
    gmmreg::CoherentPointDriftGrbf().Run(f_config);
  } else if (!strcmp(strupr((char*)method), "TPS_L2")) {
    gmmreg::TpsRegistration_L2().Run(f_config);
  } else if (!strcmp(strupr((char*)method), "TPS_KC")) {
    gmmreg::TpsRegistration_KC().Run(f_config);
  } else if (!strcmp(strupr((char*)method), "GRBF_L2")) {
    gmmreg::GrbfRegistration_L2().Run(f_config);
  } else if (!strcmp(strupr((char*)method), "GRBF_KC")) {
    gmmreg::GrbfRegistration_KC().Run(f_config);
  } else if (!strcmp(strlwr((char*)method), "rigid")) {
    gmmreg::RigidRegistration().Run(f_config);
  } else {
    print_usage();
    return -1;
  }
  return 0;
}
