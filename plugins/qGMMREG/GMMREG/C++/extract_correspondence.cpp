/*=========================================================================
  $Author: bing.jian $
  $Date: 2011-01-27 14:35:23 -0500 (Thu, 27 Jan 2011) $
  $Revision: 134 $
  =========================================================================*/

#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>

#include <vnl/algo/vnl_determinant.h>
#include <vnl/vnl_matrix.h>
#include <vnl/vnl_vector.h>

#include "gmmreg_utils.h"
#include "utils/match_utils.h"

namespace gmmreg {

template <typename T>
void ExtractMatchingPairs(
    const vnl_matrix<T>& model,
    const vnl_matrix<T>& scene,
    const T& threshold,
    vnl_matrix<T>& extracted_model,
    vnl_matrix<T>& extracted_scene) {
  vnl_matrix<T> dist;
  vnl_matrix<int> pairs;
  ComputeSquaredDistanceMatrix<T>(model, scene, dist);
  PickIndices<T>(dist, pairs, threshold*threshold);
  std::cout << "distance threshold : " << threshold << std::endl;
  int n = pairs.cols();
  int d = model.cols();
  extracted_model.set_size(n, d);
  extracted_scene.set_size(n, d);
  std::cout << "# of matched point pairs : " << n << std::endl;
  for (int j = 0; j < n; ++j) {
    extracted_model.set_row(j,model.get_row(pairs(0, j)));
  }
  for (int j = 0; j < n; ++j) {
    extracted_scene.set_row(j,scene.get_row(pairs(1, j)));
  }
}

template <typename T>
void ExtractMatchingPairs(
    const char* model_file,
    const char* scene_file,
    const T& threshold,
    const char* extracted_model_file,
    const char* extracted_scene_file) {

  std::ifstream infile1(model_file);
  vnl_matrix<T> model;
  model.read_ascii(infile1);

  std::ifstream infile2(scene_file);
  vnl_matrix<T> scene;
  scene.read_ascii(infile2);

  vnl_matrix<T> extracted_model, extracted_scene;
  ExtractMatchingPairs<T>(
      model, scene, threshold, extracted_model, extracted_scene);

  std::ofstream outfile1(extracted_model_file, std::ios_base::out);
  extracted_model.print(outfile1);

  std::ofstream outfile2(extracted_scene_file, std::ios_base::out);
  extracted_scene.print(outfile2);
}

}  // namespace gmmreg

int main(int argc, char* argv[]) {
  if (argc < 6) {
    std::cerr << "Usage: " << argv[0]
      << " modelFile sceneFile threshold extracted_model extracted_scene"
      << std::endl;
    return -1;
  }
  gmmreg::ExtractMatchingPairs<float>(
      argv[1], argv[2], atof(argv[3]), argv[4], argv[5]);
}
