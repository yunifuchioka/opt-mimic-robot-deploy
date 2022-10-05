/**
 * This is mostly copy+pasted from open dynamic robot initiative solo
 * https://raw.githubusercontent.com/open-dynamic-robot-initiative/solo/master/include/solo/common_programs_header.hpp
 */

#pragma once

#include <signal.h>  // manage the ctrl+c signal

#include <Eigen/Dense>
#include <atomic>  // thread safe flag for application shutdown management
#include <chrono>
#include <iostream>

#include "master_board_sdk/master_board_interface.h"
#include "real_time_tools/thread.hpp"
#include "real_time_tools/timer.hpp"
#include "solo8.hpp"

typedef std::chrono::high_resolution_clock Clock;

namespace solo {
/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool CTRL_C_DETECTED(false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 *
 * @param s is the id of the signal
 */
void my_handler(int) { CTRL_C_DETECTED = true; }

/**
 * @brief Enable to kill the demos cleanly with a ctrl+c
 */
void enable_ctrl_c() {
  // make sure we catch the ctrl+c signal to kill the application properly.
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  CTRL_C_DETECTED = false;
}

/**
 * @brief Usefull tool for the demos and programs in order to print data in
 * real time.
 *
 * @param v_name  is a string defining the data to print.
 * @param v the vector to print.
 */
void print_vector(std::string v_name,
                  const Eigen::Ref<const Eigen::VectorXd> v) {
  v_name += ": [";
  rt_printf("%s", v_name.c_str());
  for (int i = 0; i < v.size(); ++i) {
    rt_printf("%0.3f, ", v(i));
  }
  rt_printf("]\n");
}

/**
 * @brief like print_vector, but doesn't print the variable name in order to
 * make it suitable for piping to a csv file
 *
 * @param v the vector to print.
 */
void print_vector_csv(const Eigen::Ref<const Eigen::VectorXd> v) {
  for (int i = 0; i < v.size(); ++i) {
    rt_printf("%0.3f", v(i));
    if (i < v.size() - 1) {
      rt_printf(", ");
    }
  }
  rt_printf("\n");
}

/**
 *  @brief Helper function for saving Eigen matrix to csv
 *  author: Aleksandar Haber
 *  https://github.com/AleksandarHaber/Save-and-Load-Eigen-Cpp-Matrices-Arrays-to-and-from-CSV-files
 */
void saveData(std::string fileName, Eigen::MatrixXd matrix) {
  // https://eigen.tuxfamily.org/dox/structEigen_1_1IOFormat.html
  const static Eigen::IOFormat CSVFormat(Eigen::FullPrecision,
                                         Eigen::DontAlignCols, ", ", "\n");

  std::ofstream file(fileName);
  if (file.is_open()) {
    file << matrix.format(CSVFormat);
    file.close();
  }
}

/**
 *  @brief Helper function for reading Eigen matrix from csv
 *  author: Aleksandar Haber
 *  https://github.com/AleksandarHaber/Save-and-Load-Eigen-Cpp-Matrices-Arrays-to-and-from-CSV-files
 */
Eigen::MatrixXd openData(std::string fileToOpen) {
  // the inspiration for creating this function was drawn from here (I did NOT
  // copy and paste the code)
  // https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix

  // the input is the file: "fileToOpen.csv":
  // a,b,c
  // d,e,f
  // This function converts input file data into the Eigen matrix format

  // the matrix entries are stored in this variable row-wise. For example if we
  // have the matrix: M=[a b c
  //	  d e f]
  // the entries are stored as matrixEntries=[a,b,c,d,e,f], that is the variable
  // "matrixEntries" is a row vector later on, this vector is mapped into the
  // Eigen matrix format
  std::vector<double> matrixEntries;

  // in this object we store the data from the matrix
  std::ifstream matrixDataFile(fileToOpen);

  // this variable is used to store the row of the matrix that contains commas
  std::string matrixRowString;

  // this variable is used to store the matrix entry;
  std::string matrixEntry;

  // this variable is used to track the number of rows
  int matrixRowNumber = 0;

  while (getline(matrixDataFile,
                 matrixRowString))  // here we read a row by row of
                                    // matrixDataFile and store every line into
                                    // the string variable matrixRowString
  {
    std::stringstream matrixRowStringStream(
        matrixRowString);  // convert matrixRowString that is a string to a
                           // stream variable.

    while (getline(matrixRowStringStream, matrixEntry,
                   ','))  // here we read pieces of the stream
                          // matrixRowStringStream until every comma, and store
                          // the resulting character into the matrixEntry
    {
      matrixEntries.push_back(stod(
          matrixEntry));  // here we convert the string to double and fill in
                          // the row vector storing all the matrix entries
    }
    matrixRowNumber++;  // update the column numbers
  }

  // here we convet the vector variable into the matrix and return the resulting
  // object, note that matrixEntries.data() is the pointer to the first memory
  // location at which the entries of the vector matrixEntries are stored;
  return Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(
      matrixEntries.data(), matrixRowNumber,
      matrixEntries.size() / matrixRowNumber);
}

struct ThreadCalibrationData {
  std::shared_ptr<Solo8> robot;
  Vector8d joint_index_to_zero;

  ThreadCalibrationData(std::shared_ptr<Solo8> robot_in) {
    robot = robot_in;

    // check if calibration file exists, and set joint zero indices accordingly
    // https://www.tutorialspoint.com/the-best-way-to-check-if-a-file-exists-using-standard-c-cplusplus
    std::ifstream ifile;
    ifile.open("../config/calib_data.csv");
    if (ifile) {
      // calibration file exists
      joint_index_to_zero = openData("../config/calib_data.csv");
    } else {
      // no calibration file
      joint_index_to_zero.setZero();
    }
  }
};

}  // namespace solo
