#ifndef MOTION_HANDLER_H
#define MOTION_HANDLER_H

#include <boost/thread.hpp>

#include "Utils\ReferencePoint.h"
#include "Path_planning\Motion_sequence.h"
#include "Path_planning\Motion_sequence_gui_converter.h"
#include "Manager\Safe_type.h"
#include "Manager\Writen_paths_filenames.h"
#include "Manager\Target_configurations_manager.h"
#include "Utils\Communication_utils\socket.h"

template <typename Kernel>
class Motion_handler
{
public:
  typedef Reference_point<Kernel>     Ref_p;
  typedef Motion_sequence<Kernel>     Motion_sequence;
  typedef Motion_step_base<Kernel>    Motion_step;
  typedef Environment<Kernel>         Environment;
  
  typedef Safe_type<Ref_p>            Robot_location;

  typedef Target_configurations_manager<Kernel> Target_configurations_manager;
  
  
  
private:
  typedef std::vector<int>                         Int_vec;
  typedef Motion_sequence_gui_converter<Kernel>    Converter;
public:
  Motion_handler() {} //should be instansiated also with a 
                      //a pointer to the GUI
                      //a pointer to the planner
  void start( bool                    is_player_a,
              Environment*            env_ptr,
              Socket*                 socket_gui_ptr,
              Robot_location*         my_robot_ptr,
              Robot_location*         other_location_ptr,
              Writen_paths_filenames* writen_paths_filenames_ptr,
              Target_configurations_manager*  target_configurations_manager_ptr)
  {
    thread = boost::thread( &Motion_handler::handle_motion, this, 
                            is_player_a, env_ptr, socket_gui_ptr,
                            my_robot_ptr, other_location_ptr, writen_paths_filenames_ptr, target_configurations_manager_ptr);
    return;
  }
  void join()
  {
    thread.join();
  }
private:
  void handle_motion( bool                    is_player_a,
                      Environment*            env_ptr,            //needed for planner
                      Socket*                 socket_gui_ptr,
                      Robot_location*         my_robot_ptr,       //needed for planner
                      Robot_location*         other_location_ptr, //needed for planner
                      Writen_paths_filenames* writen_paths_filenames_ptr,
                      Target_configurations_manager*  target_configurations_manager_ptr)
  {
    std::vector<std::string> file_names_vec;
    writen_paths_filenames_ptr->pop_paths_filenames(std::back_inserter(file_names_vec));

    if (file_names_vec.empty())
      return;

    //read motion from motion file
    Motion_sequence motion_sequence;
    BOOST_FOREACH (std::string file_name, file_names_vec)
    {
      std::ifstream in(file_name.c_str());
      motion_sequence.read(in);
    }
    CGAL_precondition ( my_robot_ptr->get() == 
                        motion_sequence.get_sequence().front()->source() );

    //validate motion
    bool valid_motion = this->validate_motion_sequence(motion_sequence);
    CGAL_postcondition (valid_motion); //ToDo - change to show that path is wrong and end game

    //update location of robot
    Ref_p update_ref_p = motion_sequence.get_sequence().back()->target();
    my_robot_ptr->set(update_ref_p);

    //draw motion GUI and update located configurations
    this->draw_motion_and_update_conf_manager(is_player_a, motion_sequence, socket_gui_ptr, other_location_ptr, target_configurations_manager_ptr);
  }
  bool validate_motion_sequence(Motion_sequence& motion_sequence)
  {
    for (unsigned int i(0); i < motion_sequence.get_sequence().size(); ++i)
    {
      Motion_step* motion_step_ptr = motion_sequence.get_sequence()[i];
      if (motion_step_ptr->type() == Motion_step::TRANSLATION)
      {
        if (validate_motion_step_translational(motion_step_ptr) == false)
          return false;
      }
      else // (motion_step.type() == Motion_step<Kernel>::ROTATION)
      {
        if (validate_motion_step_rotational(motion_step_ptr) == false)
          return false;
      }
    }
    return true;
  }
  bool validate_motion_step_translational(Motion_step* motion_step_ptr)
  {
    return true; //ToDo - validate...
  }
  bool validate_motion_step_rotational(Motion_step* motion_step_ptr)
  {
    return true; //ToDo - validate...
  }
  void draw_motion_and_update_conf_manager( bool is_player_a, Motion_sequence& motion_sequence,
                                            Socket* socket_gui_ptr, Robot_location* other_location_ptr,
                                            Target_configurations_manager*  target_configurations_manager_ptr)
  {
    std::string path_file_name = this->get_path_file_name();
    std::string status_file_name = this->get_status_file_name();
    const Motion_sequence::Motion_vec& motion_vec = motion_sequence.get_sequence();
    
    //convert path & duration to GUI format and write to file
    Converter::Path_3d path;
    Converter converter;
    converter(motion_sequence, std::back_inserter(path));
    double other_x = CGAL::to_double(other_location_ptr->get().get_location().x());
    double other_y = CGAL::to_double(other_location_ptr->get().get_location().y());
    double other_t = other_location_ptr->get().get_rotation().to_angle();

    ofstream path_os(path_file_name.c_str());
    path_os <<path.size()<<std::endl;
    for (unsigned int i(0); i < path.size(); ++i)
    {
      if (i==0)
      {
        if (is_player_a)
          path_os << path[i].x  <<" " << path[i].y  <<" " << path[i].t  <<" "
                  << other_x    <<" " << other_y    <<" " << other_t ;
        else //!is_player_a
          path_os << other_x    <<" " << other_y    <<" " << other_t    <<" "
                  << path[i].x  <<" " << path[i].y  <<" " << path[i].t ;
      }
      else //(i>0)
      {
        if (is_player_a)
          path_os << path[i].x  <<" " << path[i].y  <<" " << path[i].t <<" "
                  << 0 <<" " << 0 <<" " << 0 <<" " ;
        else //!is_player_a
          path_os << 0 <<" " << 0 <<" " << 0 <<" " 
                  << path[i].x  <<" " << path[i].y  <<" " << path[i].t <<" " ;

        //add speed
        double dx = (path[i].x - path[i-1].x);
        double dy = (path[i].y - path[i-1].y);
        double dt = (path[i].t - path[i-1].t);

        double dist, speed;
        if (dt == 0) //translation
        {
          dist = CGAL::to_double(std::sqrt(dx*dx + dy*dy));
          speed = configuration.get_translational_speed();
        }
        else //rotation
        {
          dist = std::abs(dt);
          CGAL_postcondition( dist >=0 && dist <360);
          speed = configuration.get_rotational_speed();
        }

        path_os<< dist /speed;
      }
      path_os<<std::endl;      
    }
  
    //get motion status and write to file
    ofstream status_os(status_file_name.c_str());
    
    std::vector<Int_vec> configuration_statuses;
    Int_vec configuration_status;

    //get status before motion    
    target_configurations_manager_ptr->get_configuration_status(configuration_status);
    configuration_statuses.push_back(configuration_status);

    BOOST_FOREACH(Motion_sequence::MS_base_ptr ms_ptr, motion_vec)
    {
      configuration_status.clear();
      target_configurations_manager_ptr->mark_contained_configurations(ms_ptr, is_player_a);
      target_configurations_manager_ptr->get_configuration_status(configuration_status);
      configuration_statuses.push_back(configuration_status);
    }
    
    //write status to file
    status_os <<configuration_statuses.size()<<std::endl;
    for (unsigned int i(0); i < configuration_statuses.size(); ++i)
    {
      status_os <<i<<std::endl;
      for (unsigned int j(0); j < configuration_statuses[i].size(); ++j)
        status_os <<configuration_statuses[i][j]<<" ";
      status_os <<std::endl;
    }

    std::stringstream path_update;
		path_update << path_file_name << " "; //path to draw filename
		path_update << status_file_name << " "; //state to draw filename
		socket_gui_ptr->send_line(path_update.str().c_str());
    return;
  }
  
  std::string get_path_file_name()
  {
    static int i(0);
    std::stringstream path_file_name;
    path_file_name << "path_";
    path_file_name << i++;
    path_file_name << ".txt";
    return path_file_name.str();
  }
  
  std::string get_status_file_name()
  {
    static int i(0);
    std::stringstream status_file_name;
    status_file_name <<"status_";
    status_file_name << i++;
    status_file_name << ".txt";
    return status_file_name.str();
  }

private:
  boost::thread thread;
}; //Motion_handler
#endif //MOTION_HANDLER_H