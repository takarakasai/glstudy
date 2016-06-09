
#include "Utils.h"

namespace ssg {

  /* slices : num of divisions */
  std::vector<Eigen::Vector2f> circle_tbl (size_t slices) {
  
    std::vector<Eigen::Vector2f> vecs;
    vecs.reserve(slices + 1);
    
    vecs.push_back(Eigen::Vector2f(1.0, 0));
    for (size_t i = 1; i < slices; i++) {
      float rad = 2 * Dp::Math::PI * (float)i / slices;
      vecs.push_back(Eigen::Vector2f(cos(rad), sin(rad)));
      //printf("%lf %lf %lf\n", cos(rad), sin(rad), rad);
    }
    vecs.push_back(Eigen::Vector2f(1.0, 0));
  
    return vecs;
  }
}

