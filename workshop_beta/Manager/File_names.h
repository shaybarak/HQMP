#ifndef FILE_NAMES_H
#define FILE_NAMES_H

#include "Manager\Time_frame.h"
#include <boost/thread/mutex.hpp>

class File_names
{
private:
  Time_frame*     time_frame_ptr;
  std::string     additional_target_configurations_file_name;
  std::string     robot_a_location_file_name_base, robot_b_location_file_name_base;
  int             n_a, n_b, m_a, m_b;
  boost::mutex    mutex_a, mutex_b;

public:
  File_names( Time_frame* time_frame_ptr_,              
              std::string robot_a_location_file_name_base_ = "robot_a_location",
              std::string robot_b_location_file_name_base_ = "robot_b_location",
              std::string additional_target_configurations_file_name_ = "additional_target_configurations.txt") 
              : time_frame_ptr(time_frame_ptr_),
                robot_a_location_file_name_base(robot_a_location_file_name_base_),
                robot_b_location_file_name_base(robot_b_location_file_name_base_),
                additional_target_configurations_file_name(additional_target_configurations_file_name_),                
                n_a(0), n_b(0), m_a(0), m_b(0)
  {}

  const std::string& get_additional_target_configurations_file_name();
  std::string get_robot_a_location_file_name();
  std::string get_robot_b_location_file_name();
  std::string get_next_robot_a_path_file_name();
  std::string get_next_robot_b_path_file_name();
}; //File_names

#endif //FILE_NAMES_H