#include <iostream>
#include <fstream>
#include <assert.h>

#include <vnl/algo/vnl_qr.h>

#ifdef WIN32
#include <windows.h>
#else
#include "port_ini.h"
#endif

#include "gmmreg_utils.h"
#include "utils/io_utils.h"

namespace gmmreg {

int ThinPlateSplineTransform(const char* f_config) {
  vnl_matrix<double> model, scene, ctrl_pts, /*source,*/ transformed_source;
  char f_model[256] = {0}, f_scene[256] = {0};
  char common_section[80] = "FILES";

  GetPrivateProfileString(common_section, "model", NULL,
      f_model, 256, f_config);
  if (LoadMatrixFromTxt(f_model, model) < 0) {
    return -1;
  }
  int d = model.cols();

  GetPrivateProfileString(common_section, "scene", NULL,
      f_scene, 256, f_config);
  if (LoadMatrixFromTxt(f_scene, scene) < 0) {
    return -1;
  }
  assert(scene.cols() == d);

  char f_ctrl_pts[256] = {0};
  GetPrivateProfileString(common_section, "ctrl_pts", NULL,
      f_ctrl_pts, 256, f_config);
  if (LoadMatrixFromTxt(f_ctrl_pts, ctrl_pts) < 0) {
    return -1;
  }
  assert(ctrl_pts.cols() == d);
  int n = ctrl_pts.rows();

  //char f_source[256] = {0};
  //GetPrivateProfileString(common_section, "source", NULL,
  //    f_source, 256, f_config);
  //if (LoadMatrixFromTxt(f_source, source) < 0) {
  //  return -1;
  //}
  //assert(source.cols() == d);
  //int m = source.rows();

  vnl_matrix<double> affine, tps;
  char f_init_affine[256] = {0}, f_init_tps[256] = {0};
  DWORD readCount = GetPrivateProfileString(common_section, "init_affine", NULL,
      f_init_affine, 256, f_config);
  if (readCount != 0)
  {
	  if (LoadMatrixFromTxt(f_init_affine, affine) < 0) {
		  return -1;
	  }
  }
  assert(affine.cols() == d);
  assert(affine.rows() == d+1);

  GetPrivateProfileString(common_section, "init_tps", NULL,
      f_init_tps, 256, f_config);
  if (LoadMatrixFromTxt(f_init_tps, tps) < 0) {
    return -1;
  }
  assert(tps.cols() == d);
  assert(tps.rows() == (n - d - 1));

  vnl_matrix<double> param_all;
  param_all.set_size(n, d);
  param_all.update(affine);
  param_all.update(tps, d + 1);

  // TODO: check if dimensions are consistent.
  bool b_normalize = GetPrivateProfileInt("GMMREG_OPT", "normalize", 1, f_config);

  double model_scale, scene_scale, ctrl_scale;
  vnl_vector<double> ctrl_centroid, model_centroid, scene_centroid;

  if (b_normalize) {
    Normalize(ctrl_pts, ctrl_centroid, ctrl_scale);
    Normalize(model, model_centroid, model_scale);
    Normalize(scene, scene_centroid, scene_scale);
    // perform normalization to source w.r.t to model space
    //Denormalize(source, -model_centroid/model_scale, 1.0/model_scale);
  }

  vnl_matrix<double> K, U;
  ComputeTPSKernel(source, ctrl_pts, U, K);
  vnl_matrix<double> Pm;
  Pm.set_size(m, d + 1);
  Pm.set_column(0, 1);
  Pm.update(source, 0, 1);

  vnl_matrix<double> Pn;
  Pn.set_size(n, d + 1);
  Pn.set_column(0, 1);
  Pn.update(ctrl_pts, 0, 1);

  vnl_qr<double> qr(Pn);
  vnl_matrix<double> V = qr.Q();
  vnl_matrix<double> PP = V.extract(n, n - d - 1, 0, d + 1);
  vnl_matrix<double> basis;
  basis.set_size(m, n);
  basis.update(Pm);
  basis.update(U * PP, 0, d + 1);

  transformed_source = basis * param_all;
  if (b_normalize) {
    Denormalize(transformed_source, scene_centroid, scene_scale);
  }
  char f_transformed_source[256] = {0};
  GetPrivateProfileString(common_section, "transformed_source", NULL,
      f_transformed_source, 256, f_config);
  std::ofstream outfile(f_transformed_source, std::ios_base::out);

  transformed_source.print(outfile);
  return 0;
}
}  // namespace gmmreg

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " config_file" << std::endl;
    //print_usage();
    return -1;
  }
  gmmreg::ThinPlateSplineTransform(argv[1]);
  return 0;
}
