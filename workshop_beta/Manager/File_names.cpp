#include "stdafx.h"
#include "Manager\File_names.h"

const std::string& File_names::get_additional_target_configurations_file_name()
{
  return additional_target_configurations_file_name;
}

std::string File_names::get_robot_a_location_file_name()
{
  std::string output;
  mutex_a.lock();

  std::stringstream ss;
  ss << robot_a_location_file_name_base;
  ss << "_";
  ss << m_a++;
  ss << ".txt";
  
  output = ss.str();      
  
  mutex_a.unlock();
  return output;
}
 
std::string File_names::get_robot_b_location_file_name()
{
  std::string output;
  mutex_b.lock();
  
  std::stringstream ss;
  ss << robot_b_location_file_name_base;
  ss << "_";
  ss << m_b++;
  ss << ".txt";
  
  output = ss.str();      
  
  mutex_b.unlock();
  return output;
}
 
std::string File_names::get_next_robot_a_path_file_name()
{
  std::stringstream filename;
  filename << "player_";
  filename << "a_";
  filename << n_a++;
  filename << "_path.txt";

  return filename.str().c_str();
}

std::string File_names::get_next_robot_b_path_file_name()
{
  std::stringstream filename;
  filename << "player_";
  filename << "b_";
  filename << n_b++;
  filename << "_path.txt";

  return filename.str().c_str();
}