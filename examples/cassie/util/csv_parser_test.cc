#include "cassie/util/csv_parser.h"

#include <iostream>

using Eigen::MatrixXd;

int main(int argc, char *argv[]) {
  if (argc > 1) {
    std::cout << "reading file: " << argv[1] << std::endl;
    MatrixXd A = load_csv<MatrixXd>(argv[1]);
    std::cout << "returned" << std::endl;
    std::cout << "got " << A.rows() << " by " << A.cols()
              << " matrix:" << std::endl;
    std::cout << A << std::endl;
  } else {
    std::cout << "no file given" << std::endl;
  }

  return 0;
}
