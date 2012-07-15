//#include "StdAfx.h"
#include "stdafx.h"
#include "Utils\UI_utils\InputReader.h"


void Input_reader::read_configuration(const std::string& filename, Configuration& configuration)
    {
      std::string line;
      std::ifstream file (filename.c_str());

      if (!file.is_open())
      {
        std::cerr<< "unable to open filename: " <<filename.c_str()<<std::endl;
        assert (false);
      }

      while (! file.eof() )
      {        
        getline (file,line);
        if ( (line == "") || (line[0] == '#') )
          continue;

        std::vector<std::string> decomposed_line;
        string_split(line," ",decomposed_line);

        //sanity
        if (  (decomposed_line.size() < 2) || 
              ( (decomposed_line.size() > 2) && ((decomposed_line[2])[0] != '#') ))
        {
          std::cerr <<"error in following line of configuration file " <<std::endl
                    <<line.c_str()<<std::endl;
        }
        
        if (decomposed_line[0].compare("robot_file_name_a") == 0)
          configuration.set_robot_file_name_a (decomposed_line[1]);
        if (decomposed_line[0].compare("robot_file_name_b") == 0)
          configuration.set_robot_file_name_b (decomposed_line[1]);
        else if (decomposed_line[0].compare("workspace_file_name") == 0)
          configuration.set_workspace_file_name(decomposed_line[1]);
        
        else if (decomposed_line[0].compare("where_to_save_path_folder") == 0)
          configuration.set_where_to_save_path_folder(decomposed_line[1]);
        else if (decomposed_line[0].compare("save_path") == 0)
          configuration.set_save_path(atoi(decomposed_line[1].c_str()));

        else if (decomposed_line[0].compare("slices_granularity") == 0)
          configuration.set_slices_granularity(atoi(decomposed_line[1].c_str()));
        else if (decomposed_line[0].compare("use_filtering") == 0)
          configuration.set_use_filtering(atoi(decomposed_line[1].c_str()));
        else if (decomposed_line[0].compare("use_region_of_interest") == 0)
          configuration.set_use_region_of_interest(atoi(decomposed_line[1].c_str()));

        else if (decomposed_line[0].compare("rotation_range") == 0)
          configuration.set_rotation_range(atoi(decomposed_line[1].c_str()));
        else if (decomposed_line[0].compare("scale_ratio_for_small_features") == 0)
          configuration.set_rotation_range(atof(decomposed_line[1].c_str()));

        else if (decomposed_line[0].compare("generate_random_fsc_prob") == 0)
          configuration.set_generate_random_fsc_prob(atoi(decomposed_line[1].c_str()));
        else if (decomposed_line[0].compare("max_area_of_small_feature") == 0)
          configuration.set_max_area_of_small_feature(atof(decomposed_line[1].c_str()));
        else if (decomposed_line[0].compare("max_size_of_small_feature") == 0)
          configuration.set_max_size_of_small_feature(atof(decomposed_line[1].c_str()));
        else if (decomposed_line[0].compare("min_area_of_large_feature") == 0)
          configuration.set_min_area_of_large_feature(atof(decomposed_line[1].c_str()));
        else if (decomposed_line[0].compare("workspace_area") == 0)
          configuration.set_workspace_area(atof(decomposed_line[1].c_str()));

        else if (decomposed_line[0].compare("max_num_of_intra_connections") == 0)
          configuration.set_max_num_of_intra_connections(atoi(decomposed_line[1].c_str()));
        else if (decomposed_line[0].compare("max_num_of_base_intra_connections") == 0)
          configuration.set_max_num_of_base_intra_connections(atoi(decomposed_line[1].c_str()));        
        else if (decomposed_line[0].compare("max_num_of_phases") == 0)
          configuration.set_max_num_of_phases(atoi(decomposed_line[1].c_str()));        

        else if (decomposed_line[0].compare("rotational_speed") == 0)
          configuration.set_rotational_speed(atof(decomposed_line[1].c_str()));        
        else if (decomposed_line[0].compare("translational_speed") == 0)
          configuration.set_translational_speed(atof(decomposed_line[1].c_str()));        

        else if (decomposed_line[0].compare("host_name") == 0)
          configuration.set_host_name(decomposed_line[1]);        
        else if (decomposed_line[0].compare("host_port") == 0)
          configuration.set_host_port(atoi(decomposed_line[1].c_str()));

        else if (decomposed_line[0].compare("source_configuration_a_file_name") == 0)
          configuration.set_source_configuration_a_file_name(decomposed_line[1]);
        else if (decomposed_line[0].compare("source_configuration_b_file_name") == 0)
          configuration.set_source_configuration_b_file_name(decomposed_line[1]);
        else if (decomposed_line[0].compare("target_configurations_file_name") == 0)
          configuration.set_target_configurations_file_name(decomposed_line[1]);
        else if (decomposed_line[0].compare("additional_target_configurations_file_name") == 0)
          configuration.set_additional_target_configurations_file_name(decomposed_line[1]);
        else if (decomposed_line[0].compare("all_target_configurations_file_name") == 0)
          configuration.set_all_target_configurations_file_name(decomposed_line[1]);
      }
      
      file.close();
      
      return;

    }
////////////////////
//private functions/
////////////////////
void Input_reader::string_split(std::string str, const std::string delim, std::vector<std::string>& results)
{
  int cut_at;
  
  while( (cut_at = str.find_first_of(delim)) != str.npos )
  {
    if(cut_at > 0)
      results.push_back(str.substr(0, cut_at));
    str = str.substr(cut_at+1);
  }

  if(str.length() > 0)
    results.push_back(str);
  
  return;
}


void Input_reader::remove_delimiter(const std::string& in_file_name, const std::string& out_file_name, const std::string& delim)
{
  std::ifstream in_file  (in_file_name.c_str());
  std::ofstream out_file (out_file_name.c_str());

  CGAL_precondition(in_file.is_open());
  while (! in_file.eof() )
  {
    std::vector<std::string> results;
    std::string line;

    getline (in_file,line);
    string_split(line,delim,results);
    BOOST_FOREACH (std::string s , results)
      out_file << s << " ";
    out_file << std::endl;
  }
  out_file.close();
  return;
}

CGAL::Gmpq  Input_reader::to_number_type_gmpq (std::string input)
{
  CGAL::Gmpq result;
  if (input.find(".") != std::string::npos)
  {
    CGAL::Gmpq d (1);
    std::vector<std::string> results;
    string_split(input,".",results);
    CGAL::Gmpq top(results[0]);
    CGAL::Gmpq n(results[1]);
    for (unsigned int i=0; i<results[1].length(); i++)
      d *= 10;
    result = top + CGAL::Gmpq (n.numerator(),d.numerator());
    if ((input.find("-") != std::string::npos) &&
        (top == 0))
    {
      result *= -1;
    }
  }
  else
    result = CGAL::Gmpq (input);
  return result;
}

CGAL::CORE_algebraic_number_traits::Rational  Input_reader::to_number_type_core_algebraic (std::string input)
{
  typedef CGAL::CORE_algebraic_number_traits::Rational NT;
  typedef CGAL::CORE_algebraic_number_traits           Nt_traits;

  NT result;
  if (input.find(".") != std::string::npos)
  {
    std::vector<std::string> results;
    string_split(input,".",results);

  NT top(results[0]);
  NT d = 1;
  for (unsigned int i=0; i<results[1].length(); i++)
      d *= 10;
  
  std::string tmp(results[1]);
	    if (results[1].at(0) == '0') 
  {
    tmp.erase(0,tmp.find_first_not_of ('0'));
  }
  NT n(tmp);
  
  Nt_traits nt_traits;
  NT  result = top + NT (nt_traits.numerator(n),nt_traits.numerator(d));
    
    if ( (input.find("-") != std::string::npos) &&
     (top == 0) )
  {
      result *= -1;
    }
    return result;
  }
  else
  {
    result = NT (input);
    return result;
  }
}

CGAL::CORE_arithmetic_kernel::Rational    Input_reader::to_number_type_core_arithmetic(std::string input)
{
  typedef CGAL::CORE_arithmetic_kernel::Rational NT;
  typedef CGAL::CORE_arithmetic_kernel			 Nt_traits;

  NT result;
  if (input.find(".") != std::string::npos)
  {
    std::vector<std::string> results;
    string_split(input,".",results);

  NT top(results[0]);
  NT d = 1;
  for (unsigned int i=0; i<results[1].length(); i++)
      d *= 10;
  
  std::string tmp(results[1]);
	    if (results[1].at(0) == '0') 
  {
    tmp.erase(0,tmp.find_first_not_of ('0'));
  }
  NT n(tmp);
  NT  result = top + NT (CORE::numerator(n),CORE::numerator(d));
  if ((input.find("-") != std::string::npos) &&
    (top == 0))
   {
     result *= -1;
   }
  return result;
  }
  else
  {
    result = NT (input);
  return result;
  }
}