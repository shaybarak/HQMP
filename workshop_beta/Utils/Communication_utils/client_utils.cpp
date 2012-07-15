#include "stdafx.h"

#include "Utils\Communication_utils\client_utils.h"
#include <sstream>
#include <iostream>

Time_frame_status get_time_frame_status(Socket_client& socket_client)
{
  //construct request
  std::stringstream request;
  request << TIME_FRAME_STATUS_REQUEST  ;
  std::cout <<"message sent in client: "<<request.str()<<std::endl;
  socket_client.send_line(request.str());

  //recieve reply
  std::string answer = socket_client.receive_line();
  
  std::cout <<"answer in "<<"get_time_frame_status " <<answer<<std::endl;

  std::vector<std::string> decomposed_result;
  string_split(answer, " ", std::back_inserter(decomposed_result));

  //convert reply
  int header = atoi(decomposed_result[0].c_str());
  std::cout <<header<<" " <<decomposed_result[0].c_str() <<std::endl;
  CGAL_postcondition(header == TIME_FRAME_STATUS_REPLY);
  int is_moveable_int = atoi(decomposed_result[1].c_str());
  CGAL_postcondition( (is_moveable_int == 0) || (is_moveable_int == 1) );
  double remaining_time = atof(decomposed_result[2].c_str());
  
  Time_frame_status status;
  status.is_moveable    = static_cast<bool> (is_moveable_int);
  status.remaining_time = remaining_time;

  return status;
}

Scene_status get_scene_status(Socket_client& socket_client)
{
  //construct request
  std::stringstream request;
  request << SCENE_STATUS_REQUEST;
  std::cout <<"message sent in client: "<<request.str()<<std::endl;
  socket_client.send_line(request.str());

  //recieve reply
  std::string answer = socket_client.receive_line();
  std::cout <<"answer in "<<"get_scene_status " <<answer<<std::endl;


  std::vector<std::string> decomposed_result;
  string_split(answer, " ", std::back_inserter(decomposed_result));

  //convert reply
  int header = atoi(decomposed_result[0].c_str());
  CGAL_postcondition(header == SCENE_STATUS_REPLY);

  Scene_status status;
  status.quasi_dynamic_obs_location_filename = decomposed_result[1].c_str();
  
  int updated_target_configurations_int = atoi(decomposed_result[2].c_str());
  assert( (updated_target_configurations_int == 0) || 
          (updated_target_configurations_int == 1) );
    
  status.updated_target_configurations      = static_cast<bool> (updated_target_configurations_int);
  if (status.updated_target_configurations)
    status.updated_target_configurations_filename = decomposed_result[3].c_str();
  
  return status;
}

bool request_to_write(Socket_client& socket_client, double time, std::string& path_filename)
{
  //construct request
  std::stringstream request;
  request << WRITE_REQUEST << " ";
  request << time;
  socket_client.send_line(request.str());

  //recieve reply
  std::string answer = socket_client.receive_line();
  std::cout <<"answer in "<<"request_to_write " <<answer<<std::endl;

  std::vector<std::string> decomposed_result;
  string_split(answer, " ", std::back_inserter(decomposed_result));

  //convert reply
  int header = atoi(decomposed_result[0].c_str());
  CGAL_postcondition(header == WRITE_REQUEST_REPLY);
  int request_approved_int = atoi(decomposed_result[1].c_str());
  CGAL_postcondition( (request_approved_int == 0) || (request_approved_int == 1) );
  bool request_approved = static_cast<bool> (request_approved_int);
  if (request_approved)
    path_filename = decomposed_result[2];
  return request_approved;
}

void terminate_connection(Socket_client& socket_client)
{
  //construct request
  std::stringstream request;
  request << TERMINATE ;
  socket_client.send_line(request.str());

  socket_client.close();  
}