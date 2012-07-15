#include "StdAfx.h"
#include "Configuration.h"

//////////////
//	globals //
//////////////
Configuration configuration;

///////////////////
// set functions //
///////////////////
void Configuration::set_robot_file_name_a (const string& robot_file_name_a)
{
  _robot_file_name_a = robot_file_name_a;
  return;
}

void Configuration::set_robot_file_name_b (const string& robot_file_name_b)
{
  _robot_file_name_b = robot_file_name_b;
  return;
}

void Configuration::set_workspace_file_name(const std::string& workspace_file_name)
{
  _workspace_file_name = workspace_file_name;
  return;
}

void Configuration::set_where_to_save_path_folder(const std::string& where_to_save_path_folder)
{
  _where_to_save_path_folder = where_to_save_path_folder;
  return;
}
void Configuration::set_save_path(bool save_path)
{
  _save_path =save_path;
  return;
}

void Configuration::set_slices_granularity(int slices_granularity)
{
  _slices_granularity = slices_granularity;
  return;
}

void Configuration::set_use_filtering(bool use_filtering)
{
  _use_filtering = use_filtering;
  return;
}

void Configuration::set_use_region_of_interest(bool use_region_of_interest)
{
  _use_region_of_interest = use_region_of_interest;
  return;
}

void Configuration::set_rotation_range(int rotation_range)
{
  _rotation_range = rotation_range;
  return;
}

void Configuration::set_scale_ratio_for_small_features (double scale_ratio_for_small_features )
{
  _scale_ratio_for_small_features  = scale_ratio_for_small_features ;
  return;
}

void Configuration::set_max_area_of_small_feature (double max_area_of_small_feature)
{
  _max_area_of_small_feature = max_area_of_small_feature;
  return;
}
void Configuration::set_max_size_of_small_feature (double max_size_of_small_feature)
{
  _max_size_of_small_feature = max_size_of_small_feature;
  return;
}
void Configuration::set_min_area_of_large_feature (double min_area_of_large_feature)
{
  _min_area_of_large_feature = min_area_of_large_feature;
  return;
}

void Configuration::set_workspace_area(double workspace_area)
{
  _workspace_area = workspace_area;
  return;
}

void Configuration::set_generate_random_fsc_prob(double generate_random_fsc_prob)
{
  _generate_random_fsc_prob = generate_random_fsc_prob;
  return;
}

void Configuration::set_min_num_of_slices_aoi(int min_num_of_slices_aoi)
{
  _min_num_of_slices_aoi = min_num_of_slices_aoi;
  return;
}

void Configuration::set_max_num_of_intra_connections(int max_num_of_intra_connections)
{
  _max_num_of_intra_connections = max_num_of_intra_connections;
}
void Configuration::set_max_num_of_base_intra_connections(int max_num_of_base_intra_connections)
{
  _max_num_of_base_intra_connections = max_num_of_base_intra_connections;
}
void Configuration::set_max_num_of_phases(int max_num_of_phases)
{
  _max_num_of_phases = max_num_of_phases;
}

void Configuration::set_rotational_speed(double rotational_speed)
{
  _rotational_speed = rotational_speed;
}
void Configuration::set_translational_speed(double translational_speed)
{
  _translational_speed = translational_speed;
}

void Configuration::set_host_name(const std::string& host_name)
{
  _host_name = host_name;
}
void Configuration::set_host_port(int host_port)
{
  _host_port = host_port;
}


void Configuration::set_source_configuration_a_file_name(const std::string& source_configuration_a_file_name)
{
  _source_configuration_a_file_name = source_configuration_a_file_name;
}
void Configuration::set_source_configuration_b_file_name(const std::string& source_configuration_b_file_name)
{
  _source_configuration_b_file_name = source_configuration_b_file_name;
}
void Configuration::set_target_configurations_file_name(const std::string& target_configurations_file_name)
{
  _target_configurations_file_name = target_configurations_file_name;
}
void Configuration::set_additional_target_configurations_file_name(const std::string& additional_target_configurations_file_name)
{
  _additional_target_configurations_file_name = additional_target_configurations_file_name;
}
void Configuration::set_all_target_configurations_file_name(const std::string& all_target_configurations_file_name)
{
  _all_target_configurations_file_name = all_target_configurations_file_name;
}

///////////////////
// get functions //
///////////////////
const std::string& Configuration::get_robot_file_name_a()
{
  return _robot_file_name_a;
}

const std::string& Configuration::get_robot_file_name_b()
{
  return _robot_file_name_b;
}

const std::string& Configuration::get_workspace_file_name()
{
  return _workspace_file_name;
}

const std::string& Configuration::get_where_to_save_path_folder()
{
  return _where_to_save_path_folder;
}

bool  Configuration::get_save_path()
{
  return _save_path;
}

int   Configuration::get_slices_granularity()
{
  return _slices_granularity;
}

bool  Configuration::get_use_filtering()
{
  return _use_filtering;
}

bool  Configuration::get_use_region_of_interest()
{
  return _use_region_of_interest;
}

int   Configuration::get_rotation_range()
{
  return _rotation_range;
}

double Configuration::get_scale_ratio_for_small_features ()
{
  return _scale_ratio_for_small_features;
}

double Configuration::get_generate_random_fsc_prob()
{
  return _generate_random_fsc_prob;
}

int    Configuration::get_min_num_of_slices_aoi()
{
  return _min_num_of_slices_aoi;
}

double Configuration::get_max_area_of_small_feature ()
{
  return _max_area_of_small_feature;
}

double Configuration::get_max_size_of_small_feature ()
{
  return _max_size_of_small_feature;
}

double Configuration::get_min_area_of_large_feature ()
{
  return _min_area_of_large_feature;
}

double Configuration::get_workspace_area()
{
  return _workspace_area ;
}
int Configuration::get_max_num_of_intra_connections()
{
  return _max_num_of_intra_connections;
}
int Configuration::get_max_num_of_base_intra_connections()
{
  return _max_num_of_base_intra_connections;
}
int Configuration::get_max_num_of_phases()
{
  return _max_num_of_phases;
}

double Configuration::get_rotational_speed()
{
  return _rotational_speed;
}
double Configuration::get_translational_speed()
{
  return _translational_speed;
}

std::string Configuration::get_host_name()
{
  return _host_name;
}
int Configuration::get_host_port()
{
  return _host_port;
}

std::string Configuration::get_source_configuration_a_file_name()
{
  return _source_configuration_a_file_name;
}
std::string Configuration::get_source_configuration_b_file_name()
{
  return _source_configuration_b_file_name;
}
std::string Configuration::get_target_configurations_file_name()
{
  return _target_configurations_file_name;
}
std::string Configuration::get_additional_target_configurations_file_name()
{
  return _additional_target_configurations_file_name;
}
std::string Configuration::get_all_target_configurations_file_name()
{
  return _all_target_configurations_file_name;
}
