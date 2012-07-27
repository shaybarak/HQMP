#ifndef SYNCRONIZER_H
#define SYNCRONIZER_H

#include "Utils\Communication_utils\socket.h"
#include "Path_planning\Motion_sequence.h"
#include "Manager\Motion_handler.h"
#include "Manager\Safe_type.h"
#include "Manager\Target_configurations_manager.h"
#include "Manager\Additional_target_configurations_manager.h"
#include "Time_frame.h"
#include <boost/thread.hpp>

template <typename Kernel>
class Syncronizer
{
public:
  typedef Reference_point<Kernel>     Ref_p;
  typedef Motion_sequence<Kernel>     Motion_sequence;
  typedef Environment<Kernel>         Environment;
  typedef Target_configurations_manager<Kernel> Target_configurations_manager;
  
  typedef Safe_type<Ref_p>            Robot_location;
  typedef Motion_handler<Kernel>      Motion_handler;

  typedef std::pair<int, int>                         Result;
  typedef std::pair<Robot_location*, Robot_location*> Robot_location_pair;
  typedef std::pair<Writen_paths_filenames*, 
                    Writen_paths_filenames*>          Writen_paths_filenames_pair;


public:
  Syncronizer() {}
  void start( Time_frame*   time_frame,
              Environment*  env_ptr,
              Result*       result_ptr,
              Socket*       socket_gui_ptr,
              Target_configurations_manager*  target_configurations_manager_ptr,
              Additional_target_configurations_manager* additional_target_configurations_manager_ptr,
              Robot_location*                 robot_a_location_ptr,
              Robot_location*                 robot_b_location_ptr, 
              Writen_paths_filenames*         writen_paths_filenames_a_ptr,
              Writen_paths_filenames*         writen_paths_filenames_b_ptr)
  {
    Robot_location_pair         robot_location_pair(robot_a_location_ptr, 
                                                    robot_b_location_ptr);
    Writen_paths_filenames_pair writen_paths_filenames_pair(writen_paths_filenames_a_ptr, 
                                                            writen_paths_filenames_b_ptr);
    thread = boost::thread( &Syncronizer::syncronize, this, 
                          time_frame, env_ptr, result_ptr, socket_gui_ptr,
                          target_configurations_manager_ptr, additional_target_configurations_manager_ptr,
                          robot_location_pair, writen_paths_filenames_pair);
    return;
  }
  void syncronize(Time_frame* time_frame, Environment*  env_ptr, Result* result_ptr, Socket* socket_gui_ptr,
                  Target_configurations_manager*            target_configurations_manager_ptr,
                  Additional_target_configurations_manager* additional_target_configurations_manager_ptr,
                  Robot_location_pair& robot_location_pair,
                  Writen_paths_filenames_pair& writen_paths_filenames_pair)
  {
    Robot_location* robot_a_location_ptr = robot_location_pair.first;
    Robot_location* robot_b_location_ptr = robot_location_pair.second;
    Writen_paths_filenames* writen_paths_filenames_a_ptr = writen_paths_filenames_pair.first;
    Writen_paths_filenames* writen_paths_filenames_b_ptr = writen_paths_filenames_pair.second;

    time_frame->start();
    bool added_additional_configurations = false;

    //first interval
    std::cout <<"in synchronizer thread : " <<boost::this_thread::get_id()<<std::endl;
    std::cout <<"before colored_sleep"<<std::endl;
    
    if ( (added_additional_configurations == false) &&
         (additional_target_configurations_manager_ptr->added_target_configurations()) )
    {
      added_additional_configurations = true;
      target_configurations_manager_ptr->activate_additional_target_configurations();
    }

    boost::posix_time::seconds colored_sleep_time(get_colored_sleep_time());
    boost::this_thread::sleep(colored_sleep_time);

    std::cout <<"in synchronizer thread : " <<boost::this_thread::get_id()<<std::endl;
    std::cout <<"before changing color"<<std::endl;
    time_frame->change_color(get_gray_sleep_time());
    
    //begining of gray interval - safety
    //wait untill interval is over and change color
    std::cout <<"in synchronizer thread : " <<boost::this_thread::get_id()<<std::endl;
    std::cout <<"before gray sleep"<<std::endl;
    boost::posix_time::seconds gray_sleep_time(get_gray_sleep_time());
    boost::this_thread::sleep(gray_sleep_time);

    std::cout <<"in synchronizer thread : " <<boost::this_thread::get_id()<<std::endl;
    std::cout <<"before changing color"<<std::endl<<std::endl<<std::endl;
    time_frame->change_color(get_colored_sleep_time());

    while(1)
    {
      //begining of colored interval
      //(1)   check if to activate additional target configurations
      //(2)   read paths that the player whose interval ended has writen
      //      validate them and update the GUI
      //(3)   wait untill interval is over and change color

      //(1)
      if ( (added_additional_configurations == false) &&
         (additional_target_configurations_manager_ptr->added_target_configurations()) )
      {
        added_additional_configurations = true;
        target_configurations_manager_ptr->activate_additional_target_configurations();
      }
      
      // (2)
      Time_frame::Interval_color player_b_color = PLAYER_B_COLOR;
      bool is_prev_player_a (time_frame->get_color() == player_b_color);
      Motion_handler motion_handler;
      if (is_prev_player_a)
        motion_handler.start( true, env_ptr, socket_gui_ptr,
                              robot_a_location_ptr, robot_b_location_ptr,
                              writen_paths_filenames_a_ptr,
                              target_configurations_manager_ptr);
      else //is_player_b
        motion_handler.start(false, env_ptr, socket_gui_ptr,
                              robot_b_location_ptr, robot_a_location_ptr,
                              writen_paths_filenames_b_ptr,
                              target_configurations_manager_ptr);
      
      if(target_configurations_manager_ptr->completed_task(is_prev_player_a))
      {
        result_ptr->first = target_configurations_manager_ptr->get_score(true);
        result_ptr->second = target_configurations_manager_ptr->get_score(false);

        std::cout <<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"<<std::endl;
        std::cout <<"&&&          GAME    OVER           &&&"<<std::endl;
        std::cout <<"&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&"<<std::endl;
        return;
      }

      // (3)
      std::cout <<"in synchronizer thread : " <<boost::this_thread::get_id()<<std::endl;
      std::cout <<"before colored_sleep"<<std::endl;
      boost::posix_time::seconds colored_sleep_time(get_colored_sleep_time());
      boost::this_thread::sleep(colored_sleep_time);

      std::cout <<"in synchronizer thread : " <<boost::this_thread::get_id()<<std::endl;
      std::cout <<"before changing color"<<std::endl;
      time_frame->change_color(get_gray_sleep_time());
      
      //begining of gray interval - safety
      //wait untill interval is over and change color
      std::cout <<"in synchronizer thread : " <<boost::this_thread::get_id()<<std::endl;
      std::cout <<"before gray sleep"<<std::endl;
      boost::posix_time::seconds gray_sleep_time(get_gray_sleep_time());
      boost::this_thread::sleep(gray_sleep_time);

      std::cout <<"in synchronizer thread : " <<boost::this_thread::get_id()<<std::endl;
      std::cout <<"before changing color"<<std::endl<<std::endl<<std::endl;
      time_frame->change_color(get_colored_sleep_time());
    }

    //will not be reached
    return ;
  }
  void join()
  {
    thread.join();
  }
public:
  double get_colored_sleep_time()
  {
    return 5;
  }
  double get_gray_sleep_time()
  {
    return 5;
  }
private:
  boost::thread thread;
}; //Syncronizer

#endif //SYNCRONIZER_H