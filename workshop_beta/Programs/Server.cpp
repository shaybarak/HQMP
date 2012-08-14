#include "stdafx.h"
#include "Programs\Server.h"
#include "Utils\Communication_utils\socket.h"
#include "Utils\Communication_utils\socket_typedefs.h"

#include "Manager\File_names.h"
#include "Manager\Syncronizer.h"
#include "Manager\Time_frame.h"
#include "Manager\Safe_type.h"
#include "Manager\Request_handler.h"
#include "Manager\Additional_target_configurations_manager.h"
#include "Manager\Target_configurations_manager.h"

#include <boost/thread/mutex.hpp>

#define USE_PLAYER_A 1
#define USE_PLAYER_B 1

typedef Environment<>                         Env;
typedef Safe_type<Env::Reference_point>       Robot_location;
typedef Safe_type<bool>						  Safe_bool;
typedef Request_handler<Env::K>               Client_request_handler;
typedef Target_configurations_manager<Env::K> Targets_manager;
typedef std::pair<int, int>                   Result;
typedef std::vector<int>                      Int_vec;

Socket* set_up_connection__gui( Robot_location& robot_location_a, 
                                Robot_location& robot_location_b,
                                Targets_manager* target_configurations_manager_ptr)
{
  int host_port_g = 1103;
  int conenctions = 1;

  std::cout <<"in main thread : " <<boost::this_thread::get_id()<<std::endl;
  std::cout <<"before connecting to gui "<<std::endl<<std::endl;

  Socket_server server_g(host_port_g, conenctions);
  //here gui should be executed
  Socket* socket_g=server_g.accept();
  std::cout <<"connected to gui"<<std::endl;

  std::stringstream initial_command;
  initial_command << configuration.get_workspace_file_name()<<" ";  //workspace filename
  initial_command << configuration.get_robot_file_name_a()<<" ";    //robot filename
  initial_command << configuration.get_all_target_configurations_file_name()<<" "; //query filename
  socket_g->send_line(initial_command.str().c_str());

  //now send intial placements of each robot
  double x_a = CGAL::to_double(robot_location_a.get().get_location().x());
  double y_a = CGAL::to_double(robot_location_a.get().get_location().y());
  double t_a = robot_location_a.get().get_rotation().to_angle();

  double x_b = CGAL::to_double(robot_location_b.get().get_location().x());
  double y_b = CGAL::to_double(robot_location_b.get().get_location().y());
  double t_b = robot_location_b.get().get_rotation().to_angle();
  
  std::string path_file_name("initial_location.txt");
  ofstream path_os(path_file_name.c_str());
  path_os << 2 <<std::endl;
  //initial location
  path_os << x_a <<" " << y_a <<" " << t_a <<" "
          << x_b <<" " << y_b <<" " << t_b <<std::endl;
  //no movement
  path_os << 0 <<" " << 0 <<" " << 0 <<" "
          << 0 <<" " << 0 <<" " << 0 <<" " << 0 <<" " << std::endl;
  
  //get motion status and write to file
  std::string status_file_name("initial_status.txt");
  ofstream status_os(status_file_name.c_str());

  //get status 
  Int_vec configuration_status;  
  target_configurations_manager_ptr->get_configuration_status(configuration_status);
  
  //write status to file
  status_os <<"2" <<std::endl;  //one entry
  status_os <<0<<std::endl;     //before motion
  for (unsigned int i(0); i < configuration_status.size(); ++i)
    status_os <<configuration_status[i]<<" ";
  status_os <<1<<std::endl;     //after motion
  for (unsigned int i(0); i < configuration_status.size(); ++i)
    status_os <<configuration_status[i]<<" ";
  status_os <<std::endl;

  std::stringstream path_update;
	path_update << path_file_name << " "; //path to draw filename
	path_update << status_file_name << " "; //state to draw filename
	socket_g->send_line(path_update.str().c_str());

  return socket_g;
}
void end_connection__gui(Socket* socket)
{
  socket->send_line("CLOSE SOCKET");
  boost::this_thread::sleep(boost::posix_time::seconds(5));
  delete socket;
}


Socket* set_up_connection__player_a()
{
  int host_port_a = 1101;
  int conenctions = 1;

  std::cout <<"in main thread : " <<boost::this_thread::get_id()<<std::endl;
  std::cout <<"before connecting to client a"<<std::endl<<std::endl;

  Socket_server server(host_port_a, conenctions);
  //here program a should be executed
  Socket* socket=server.accept();

  std::cout <<"connected to player a"<<std::endl;

  return socket;
}

Socket* set_up_connection__player_b()
{
  int host_port_b = 1102;
  int conenctions = 1;

  std::cout <<"in main thread : " <<boost::this_thread::get_id()<<std::endl;
  std::cout <<"before connecting to client b"<<std::endl<<std::endl;

  Socket_server server(host_port_b, conenctions);
  //here program b should be executed
  Socket* socket=server.accept();

  std::cout <<"connected to player b"<<std::endl;

  return socket;
}

void server_main(int argc, char* argv[])
{
  ///////////////////////////////////////////////////////////////
  //set up global data structures
  Syncronizer<Env::K>                       syncronizer;
  std::cout <<"before env"<<std::endl;
  Env                                       env(argc,argv);
  std::cout <<"after env"<<std::endl;


  Time_frame::Interval_color                player_a_color = PLAYER_A_COLOR;
  Time_frame::Interval_color                player_b_color = PLAYER_B_COLOR;
  Time_frame                                time_frame( player_a_color, //player a is red
                                                        syncronizer.get_colored_sleep_time());
  File_names                                file_names(&time_frame);
  Targets_manager                           target_configurations_manager(env.get_target_configurations(),
                                                                          env.get_additional_target_configurations());
  Additional_target_configurations_manager  additional_target_configurations_manager(5,&time_frame); //add additional targets at fifth interval
                                                                                    
  Robot_location                            robot_location_a(env.get_source_configuration_a());
  Robot_location                            robot_location_b(env.get_source_configuration_b());
  Writen_paths_filenames                    writen_paths_filenames_a;
  Writen_paths_filenames                    writen_paths_filenames_b;

  Safe_bool									is_game_over(false);
  Result                                    result;

  ///////////////////////////////////////////////////////////////
  //set up sockets 
  Socket* socket_g= set_up_connection__gui(robot_location_a, robot_location_b, &target_configurations_manager);
#if USE_PLAYER_A
  Socket* socket_a= set_up_connection__player_a();
#endif //USE_PLAYER_A
#if USE_PLAYER_B   
  Socket* socket_b= set_up_connection__player_b();   
#endif //USE_PLAYER_B

  ///////////////////////////////////////////////////////////////
  //set up request handlers
#if USE_PLAYER_A
  Client_request_handler request_handler_a;
  request_handler_a.start(socket_a, player_a_color,
                          &time_frame, &file_names,
                          &additional_target_configurations_manager,
                          &writen_paths_filenames_a,
                          &robot_location_b,
						  &is_game_over);
  
#endif //USE_PLAYER_A
#if USE_PLAYER_B
  Client_request_handler request_handler_b;
  request_handler_b.start(socket_b, player_b_color,
                          &time_frame, &file_names,
                          &additional_target_configurations_manager,
                          &writen_paths_filenames_b,
                          &robot_location_a,
						  &is_game_over);
#endif //USE_PLAYER_B

  ///////////////////////////////////////////////////////////////
  //start up syncronizer

  std::cout <<"in main thread : " <<boost::this_thread::get_id()<<std::endl;
  std::cout <<"before synchronizer"<<std::endl<<std::endl;

  syncronizer.start(&time_frame, &env, &result, socket_g,
                    &target_configurations_manager,
                    &additional_target_configurations_manager,
                    &robot_location_a, &robot_location_b,
                    &writen_paths_filenames_a, &writen_paths_filenames_b,
					&is_game_over);
  
  syncronizer.join();
  
  std::cout <<"player a score: " 
            <<result.first 
            <<"player b score: "
            <<result.second 
            <<std::endl;
  
#if USE_PLAYER_A
  request_handler_a.join();
#endif //USE_PLAYER_A
#if USE_PLAYER_B
  request_handler_b.join();
#endif //USE_PLAYER_B

  end_connection__gui(socket_g);
  
  return ;
}