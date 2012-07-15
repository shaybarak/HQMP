#ifndef CONFIGURATION_H
#define CONFIGURATION_H

class Configuration
{
public: //constructors
  Configuration() {}
public: //set
  void set_robot_file_name_a (const std::string& robot_file_name_a);
  void set_robot_file_name_b (const std::string& robot_file_name_b);
  void set_workspace_file_name(const std::string& workspace_file_name);
  
  void set_where_to_save_path_folder(const std::string& where_to_save_path_folder);
  void set_save_path(bool save_path);

  void set_slices_granularity(int slices_granularity);
  void set_use_filtering(bool use_filtering);
  void set_use_region_of_interest(bool use_region_of_interest);

  void set_rotation_range(int rotation_range);
  void set_scale_ratio_for_small_features (double scale_ratio_for_small_features);

  void set_generate_random_fsc_prob(double generate_random_seg_prob);
  void set_min_num_of_slices_aoi(int min_num_of_slices_aoi);

  void set_max_area_of_small_feature (double max_area_of_small_feature);
  void set_max_size_of_small_feature (double max_size_of_small_feature);
  void set_min_area_of_large_feature (double min_area_of_large_feature);
  void set_workspace_area(double workspace_area);

  void set_max_num_of_intra_connections(int max_num_of_intra_connections);
  void set_max_num_of_base_intra_connections(int max_num_of_base_intra_connections);
  void set_max_num_of_phases(int max_num_of_phases);

  void set_rotational_speed(double rotational_speed);
  void set_translational_speed(double translational_speed);

  void set_host_name(const std::string& host_name);
  void set_host_port(int host_port);

  void set_source_configuration_a_file_name(const std::string& source_configuration_a_file_name);
  void set_source_configuration_b_file_name(const std::string& source_configuration_b_file_name);
  void set_target_configurations_file_name(const std::string& target_configurations_file_name);
  void set_additional_target_configurations_file_name(const std::string& additional_target_configurations_file_name);
  void set_all_target_configurations_file_name(const std::string& all_target_configurations_file_name);
public: //get
  const std::string& get_robot_file_name_a();
  const std::string& get_robot_file_name_b();
  const std::string& get_workspace_file_name();
  
  const std::string& get_where_to_save_path_folder();
  bool  get_save_path();

  int   get_slices_granularity();
  bool  get_use_filtering();
  bool  get_use_region_of_interest();

  int   get_rotation_range();
  double get_scale_ratio_for_small_features ();

  double get_generate_random_fsc_prob();
  int    get_min_num_of_slices_aoi();

  double get_max_area_of_small_feature ();
  double get_max_size_of_small_feature ();
  double get_min_area_of_large_feature ();
  double get_workspace_area ();

  int get_max_num_of_intra_connections();
  int get_max_num_of_base_intra_connections();
  int get_max_num_of_phases();

  double get_rotational_speed();
  double get_translational_speed();

  std::string get_host_name();
  int         get_host_port();

  std::string get_source_configuration_a_file_name();
  std::string get_source_configuration_b_file_name();
  std::string get_target_configurations_file_name();
  std::string get_additional_target_configurations_file_name();
  std::string get_all_target_configurations_file_name();

private:
  std::string _robot_file_name_a;
  std::string _robot_file_name_b;
  std::string _workspace_file_name;
  
  std::string _where_to_save_path_folder;
  bool        _save_path;
  
  int         _slices_granularity;
  bool        _use_filtering;
  bool        _use_region_of_interest;

  int         _rotation_range;
  double      _scale_ratio_for_small_features;

  double      _generate_random_fsc_prob;
  int         _min_num_of_slices_aoi;

  double      _max_area_of_small_feature;
  double      _max_size_of_small_feature;
  double      _min_area_of_large_feature;
  double      _workspace_area;

  int         _max_num_of_intra_connections;
  int         _max_num_of_base_intra_connections;
  int         _max_num_of_phases;

  double      _rotational_speed;
  double      _translational_speed;

  std::string _host_name;
  int         _host_port;

  std::string _source_configuration_a_file_name;
  std::string _source_configuration_b_file_name;
  std::string _target_configurations_file_name;
  std::string _additional_target_configurations_file_name;
  std::string _all_target_configurations_file_name;
}; //Configuration

/////////////////////////////
// Configuration manager   //
/////////////////////////////
extern Configuration configuration;

#endif //CONFIGURATION_H
