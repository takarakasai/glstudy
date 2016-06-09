
#ifndef UTILS_H
#define UTILS_H

#include <vector>

#include <eigen3/Eigen/Core>

#include "dp_type.h"

namespace ssg {

  /* slices : num of divisions */
  std::vector<Eigen::Vector2f> circle_tbl (size_t slices);

}

#endif

