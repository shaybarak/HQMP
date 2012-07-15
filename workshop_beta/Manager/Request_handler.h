#ifndef REQUEST_HANDLER_H
#define REQUEST_HANDLER_H

#include "Utils\Communication_utils\socket.h"
#include "Utils\Communication_utils\socket_typedefs.h"

#include "Manager\Time_frame.h"
#include "Manager\Writen_paths_filenames.h"
#include "Manager\Additional_target_configurations_manager.h"
#include <boost/thread/mutex.hpp>

template <typename K>
class Request_handler
{
public:
  typedef Reference_point<K>                Reference_point;
  typedef Safe_type<Reference_point>        Robot_location;
  typedef Time_frame::Interval_color        Color;
private:
struct Request_handler_data
{
  Socket*         socket_ptr;               //socket for client
  Color           player_color;             //player color

  Time_frame*                               time_frame_ptr;  
  File_names*                               file_names_ptr;
  Additional_target_configurations_manager* additional_target_configurations_manager_ptr;
  Writen_paths_filenames*                   writen_paths_filenames_ptr;
  Robot_location*                           other_robot_location_ptr;
  
}; //Request_handler_data
  
public:
  Request_handler() {}
  void start( Socket*                                   socket_ptr,
              Color                                     color,

              Time_frame*                               time_frame_ptr,
              File_names*                               file_names_ptr,
              Additional_target_configurations_manager* additional_target_configurations_manager_ptr,
              Writen_paths_filenames*                   writen_paths_filenames_ptr,
              Robot_location*                           other_robot_location_ptr)
  {
    thread = boost::thread( &Request_handler::handle_client_requests, this, 
                            socket_ptr, color, time_frame_ptr, file_names_ptr, 
                            additional_target_configurations_manager_ptr,
                            writen_paths_filenames_ptr, other_robot_location_ptr);
    return;  
  }
  void handle_client_requests(Socket*                                   socket_ptr,
                              Color                                     color,

                              Time_frame*                               time_frame_ptr,
                              File_names*                               file_names_ptr,
                              Additional_target_configurations_manager* additional_target_configurations_manager_ptr,
                              Writen_paths_filenames*                   writen_paths_filenames_ptr,
                              Robot_location*                           other_robot_location_ptr)
  {  
    while(true)    
    {
      std::string r = socket_ptr->receive_line();
      std::cout <<"message recieved in server : "<< r <<std::endl;
      std::vector<std::string> decomposed_string;
      string_split(r, " ", std::back_inserter(decomposed_string));

      int message_type(atoi(decomposed_string[0].c_str()));
      switch(message_type)
      {
      case TIME_FRAME_STATUS_REQUEST:
        {
          ////////////////////////////////////////////////////
          //  structure:
          //    (1) - header : TIME_FRAME_STATUS_REPLY
          //    (2) - fields : is_moveable
          //                 : remaining_time

          //construct reply
          bool is_moveable = (time_frame_ptr->get_color() == 
                              color);
          double remaining_time = time_frame_ptr->remaining_time();

          std::stringstream reply;
          reply << TIME_FRAME_STATUS_REPLY << " ";  //header        
          reply << is_moveable << " ";              //is_moveable
          reply << remaining_time ;                 //remaining_time

          socket_ptr->send_line(reply.str());
          break;
        }
      case SCENE_STATUS_REQUEST:
        {
          ////////////////////////////////////////////////////
          //  structure:
          //    (1) - header : SCENE_STATUS_REPLY
          //    (2) - fields : quasi_dynamic_obs_location_filename
          //                 : are there additional target configurations
          //                 : additional_target_configurations_filename

          //construct reply
          Time_frame::Interval_color player_a_color = PLAYER_A_COLOR;
          bool is_player_a (color == player_a_color);
          
          std::string quasi_dynamic_obs_location_filename = is_player_a ? 
            file_names_ptr->get_robot_a_location_file_name() :
            file_names_ptr->get_robot_b_location_file_name();
          std::ofstream os(quasi_dynamic_obs_location_filename.c_str());
          other_robot_location_ptr->get().write(os);


          bool added_target_configurations =  additional_target_configurations_manager_ptr->
                                                added_target_configurations();
          
          std::stringstream reply;
          reply << SCENE_STATUS_REPLY << " ";                   //header    
          reply << quasi_dynamic_obs_location_filename<< " ";   //filename of where the update location of the quasi dynamic obstacle is
          reply << added_target_configurations << " ";          //are there new target configurations 
          if (added_target_configurations)
            reply << file_names_ptr->get_additional_target_configurations_file_name(); //filename of where the update configurations are
          
          socket_ptr->send_line(reply.str());
          break;
        }
      case WRITE_REQUEST:
        {
          ////////////////////////////////////////////////////
          //  structure:
          //    (1) - header : WRITE_REQUEST_REPLY
          //    (2) - fields : request_approved
          //                 : motion_path_file_name

          double motion_length(atof(decomposed_string[1].c_str()));
          //can only write if it is in my interval and there is enough time
          bool is_moveable = (time_frame_ptr->get_color() == 
                              color);
          double remaining_time = time_frame_ptr->remaining_time();
          bool has_enough_time = (remaining_time >= motion_length);
          bool request_approved = has_enough_time && is_moveable;

          std::cout <<"### request recieved of time "
                    << motion_length
                    <<" there is "
                    <<remaining_time
                    <<" remaining_time "
                    <<"thus request status is : "
                    <<request_approved
                    <<std::endl;

          //construct reply
          std::stringstream reply;
          reply << WRITE_REQUEST_REPLY << " ";  //header        
          reply << request_approved << " ";     //request_approved
          
          if (request_approved)
          {
            std::string motion_path_file_name;
            CGAL_precondition (color != Time_frame::GRAY);
            Time_frame::Interval_color player_a_color = PLAYER_A_COLOR;
            bool is_player_a (color == player_a_color);
            motion_path_file_name = ( is_player_a ? 
                                      file_names_ptr->get_next_robot_a_path_file_name():
                                      file_names_ptr->get_next_robot_b_path_file_name());
            reply << motion_path_file_name ;    //filename 

            //update writen_paths
            writen_paths_filenames_ptr->push_paths_filename(motion_path_file_name);
          }
          

          socket_ptr->send_line(reply.str());
          break;
        }
      case TERMINATE:
        {
          std::cout <<std::endl<<std::endl<<std::endl<<"terminating connection..."<<std::endl<<std::endl<<std::endl;
          delete socket_ptr;
          return;
        }
      default:
        {
          //ignore
        }
      }    
    }
    return;
  }

  void join()
  {
    thread.join();
  }

private:
  boost::thread thread;
}; //Request_handler

#endif //REQUEST_HANDLER_H