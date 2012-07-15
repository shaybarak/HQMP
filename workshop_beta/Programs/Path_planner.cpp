#include "stdafx.h"
#include "Programs\Path_planner.h"

#include "Utils\Rotation_utils\Rotation.h"
#include "Utils\ReferencePoint.h"

#include "Utils\UI_utils\Environment.h"
#include "Utils\UI_utils\TimeManager.h"
#include "Utils\UI_utils\Graphics.h"

#include "Utils\Number_utils\AK_conversions_1.h"
#include "Utils\Polygon_utils\PolygonIO.h"
#include "Path_planning\Motion_sequence.h"
#include "Path_planning\Motion_sequence_gui_converter.h"
#include "Mms_example.h"


void display_scene(int argc, char* argv[])
{
  //typedefs
  typedef mms::Mms_path_planner_example<>   Planner;

  //loading files from configuration file
  Time_manager tm;
  tm.write_time_log(std::string("start"));
  
  Environment<> env(argc,argv);
  tm.write_time_log(std::string("set environment"));

  //loading scene from environment
  Planner::Polygon_vec&       workspace(env.get_workspace());
  Planner::Extended_polygon   my_robot(env.get_robot_a());	

  //load query
  Planner::Reference_point q_s (env.get_source_configuration_a());
  Planner::Reference_point q_t (env.get_target_configurations().front());

  global_graphics->draw_polygons<Planner::K> (workspace, BLUE);
  my_robot.move_absolute(q_s);
  global_graphics->draw_polygon<Planner::K> (my_robot.get_absolute_polygon(), GREEN);
  my_robot.move_absolute(q_t);
  global_graphics->draw_polygon<Planner::K> (my_robot.get_absolute_polygon(), RED);
  global_graphics->display();
  return;
}
void single_robot_planner_example(int argc, char* argv[])
{
  //typedefs
  typedef mms::Mms_path_planner_example<>             Planner;
  typedef Motion_sequence_gui_converter<Planner::K>   Converter;

  //loading files from configuration file
  Time_manager tm;
  tm.write_time_log(std::string("start"));
  
  Environment<> env(argc,argv);
  tm.write_time_log(std::string("set environment"));

  //loading scene from environment
  Planner::Polygon_vec&       workspace(env.get_workspace());
  Planner::Extended_polygon   my_robot(env.get_robot_a());	
  
  //initialize the planner and preprocess 
  Planner planner(workspace, my_robot);    
  planner.preprocess(); 
   
  //load query
  Planner::Reference_point q_s (env.get_source_configuration_a());
  Planner::Reference_point q_t (env.get_target_configurations().front());

  //perform query
  Motion_sequence<Planner::K> motion_sequence;
  bool found_path = planner.query(q_s, q_t, motion_sequence);

  if (!found_path)
    std::cout<<"no path found :-("<<std::endl;
  else
    std::cout<<"path found :-)"<<std::endl;

  //example of how to create a path that can be loaded by the GUI
  if (found_path)
  {
    Converter::Path_3d path;
    Converter converter;
    converter(motion_sequence, std::back_inserter(path));

    ofstream os("path.txt");
    os <<path.size()<<std::endl;
    for (unsigned int i(0); i < path.size(); ++i)
    {
      if (i == 0)
      {

        os  << path[i].x <<" " << path[i].y <<" " << path[i].t <<" "
            << CGAL::to_double(q_t.get_location().x()) <<" " //we place the second robot at the target for ease of visualization
            << CGAL::to_double(q_t.get_location().y()) <<" " //we place the second robot at the target for ease of visualization
            << q_t.get_rotation().to_angle()                 //we place the second robot at the target for ease of visualization
            <<std::endl;
      }
      else //i >0
      {
        os  << path[i].x <<" " << path[i].y <<" " << path[i].t <<" "  //first robot location
            << 0 <<" " << 0 <<" " << 0 <<" "                          //second robot is static
            << 2                                                      //speed is arbitrarily chosen to be 2
            <<std::endl;
      }
    }
  }
  return;
}