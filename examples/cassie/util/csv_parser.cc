#include "cassie/util/csv_parser.h"

#include <stdexcept>
#include <string>

using Eigen::Map;
using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::RowMajor;

template <typename M> M load_csv(const std::string &path) {
  // read each cell one at a time and add to a vector
  std::ifstream indata;
  indata.open(path);
  std::string line;
  std::vector<typename M::Scalar> values;
  uint rows = 0;
  while (std::getline(indata, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    while (std::getline(lineStream, cell, ',')) {
      if (std::is_same<typename M::Scalar, float>::value)
        values.push_back(std::stof(cell));
      else if (std::is_same<typename M::Scalar, double>::value)
        values.push_back(std::stod(cell));
    }
    ++rows;
  }

  // stop if there is no data
  if (values.data() == NULL) {
    throw std::runtime_error("could not read matrix from CSV");
  }

  // put the data into a matrix
  return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime,
                          M::ColsAtCompileTime, RowMajor>>(
      values.data(), rows, values.size() / rows);
}

// explicit template instantiation
// add more if the matrix size is known at compile time
template MatrixXd load_csv<MatrixXd>(const std::string &);
template MatrixXf load_csv<MatrixXf>(const std::string &);
