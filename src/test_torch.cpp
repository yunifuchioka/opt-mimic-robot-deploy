#include <iostream>

#include "solo8.hpp"
#include "network_controller.hpp"

int main(int argc, char** argv) {
std::shared_ptr<Solo8> robot = std::make_shared<Solo8>();
  NetworkController my_network_controller(robot);
  my_network_controller.initialize_network("05-09-phase-squat");

  my_network_controller.set_time(0.0);

  srand((unsigned int)time(0));  // initialize random seed
  my_network_controller.calc_control();

  NetworkController::VectorAction action = my_network_controller.get_desired_positions();


  std::cout << std::endl;
  std::cout << action.transpose() << std::endl;

  return 0;
}