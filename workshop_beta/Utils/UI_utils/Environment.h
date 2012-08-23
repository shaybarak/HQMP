#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "Utils\Polygon_utils\ExtendedPolygon.h"
#include "Utils\UI_utils\Graphics.h"
#include "Utils\UI_utils\InputReader.h"

template <typename _K = Rational_kernel>
class Environment
{
public:
  typedef _K                                K;
  typedef std::vector<CGAL::Polygon_2<K> >  Polygon_vector;
  typedef Extended_polygon<K>               Extended_polygon;
  
  typedef Reference_point<K>                Reference_point;
  typedef std::vector<Reference_point>      Reference_point_vec;

private:
  Graphics              _graphics;
  
  Polygon_vector        _workspace;
  Extended_polygon      _robot_a;
  Extended_polygon      _robot_b;

  Reference_point       _source_configuration_a;
  Reference_point       _source_configuration_b;
  Reference_point_vec   _target_configurations;
  Reference_point_vec   _additional_target_configurations;

  Reference_point_vec   _additional_sample_points;

public:
 
  Environment (int argc, char* argv[])
    :_graphics(argc,argv)
  {
    Input_reader reader;
    reader.read_configuration(argv[1], configuration); //read configuration file

    //init workspace
    std::string workspace_file_name = configuration.get_workspace_file_name();
    reader.read_polygons<K> (workspace_file_name,_workspace);
    
    //read robot a
    std::string robot_file_name_a = configuration.get_robot_file_name_a();
    CGAL::Polygon_2<K> tmp_robot_a;
    reader.read_polygon<K>(robot_file_name_a, tmp_robot_a);
    _robot_a = Extended_polygon (tmp_robot_a);

    //read robot b
    std::string robot_file_name_b = configuration.get_robot_file_name_b();
    CGAL::Polygon_2<K> tmp_robot_b;
    reader.read_polygon<K>(robot_file_name_b, tmp_robot_b);
    _robot_b = Extended_polygon (tmp_robot_b);

    ////////////////////////////////////////////////////////////////////////////
    //used for workshop
    ////////////////////////////////////////////////////////////////////////////
    //read initial configuration for robot a and b    
    std::string source_configuration_a_file_name = configuration.get_source_configuration_a_file_name();
    _source_configuration_a = reader.read_reference_point<K> (source_configuration_a_file_name);
    
    std::string source_configuration_b_file_name = configuration.get_source_configuration_b_file_name();
    _source_configuration_b = reader.read_reference_point<K> (source_configuration_b_file_name);
    
    std::string target_configurations_file_name = configuration.get_target_configurations_file_name();
    reader.read_reference_points<K> (target_configurations_file_name, std::back_inserter(_target_configurations));

    std::string additional_target_configurations_file_name = configuration.get_additional_target_configurations_file_name();
    reader.read_reference_points<K> (additional_target_configurations_file_name, std::back_inserter(_additional_target_configurations));

	//noam: additions for our group
	std::string additional_sample_points_file_name = configuration.get_additional_sample_points_file_name();
    reader.read_reference_points<K> (additional_sample_points_file_name, std::back_inserter(_additional_sample_points));


  }

  //draw functions
  void draw_workspace(QColor& color = QColor(0,0,255))
  {
    _graphics.draw_polygons<K>(_workspace, color);
  }
  void display()
  {
    _graphics.display();
  }
  //get functions
  Polygon_vector&       get_workspace() {return _workspace;}
  Extended_polygon&     get_robot_a()   {return _robot_a;}
  Extended_polygon&     get_robot_b()   {return _robot_b;}
  Reference_point&      get_source_configuration_a() {return _source_configuration_a;}
  Reference_point&      get_source_configuration_b() {return _source_configuration_b;}
  Reference_point_vec&  get_target_configurations() {return _target_configurations;}
  Reference_point_vec&  get_additional_target_configurations () {return _additional_target_configurations;}

  Reference_point_vec&  get_additional_sample_points () {return _additional_sample_points;}

}; //Environment

#endif //ENVIRONMENT_H

