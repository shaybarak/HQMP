#ifndef SOCKET_TYPEDEFS_H
#define SOCKET_TYPEDEFS_H

struct Time_frame_status
{
  bool is_moveable;     //is the robot allowed to move in the current timeframe
  double remaining_time;  //ramaining time (in seconds) for the current timeframe
}; //Time_frame_status

struct Scene_status
{
  std::string quasi_dynamic_obs_location_filename;          //filename of where the update location of the quasi dynamic obstacle is
  bool updated_target_configurations;                       //are there new target configurations 
  std::string updated_target_configurations_filename;       //filename of where the update configurations are
  
}; //Scene_status

enum Message_type 
{
  TIME_FRAME_STATUS_REQUEST,
  TIME_FRAME_STATUS_REPLY,
  SCENE_STATUS_REQUEST,
  SCENE_STATUS_REPLY,
  WRITE_REQUEST,
  WRITE_REQUEST_REPLY,
  TERMINATE
}; //Message_type

//this should move from here
template<typename OutputIterator>
void string_split(std::string str, 
                  std::string delim, 
                  OutputIterator& oi)
{
  int cut_at;
  
  while( (cut_at = str.find_first_of(delim)) != str.npos )
  {
    if(cut_at > 0)
    {
      std::string curr(str.substr(0, cut_at));
      *oi++ = curr;
    }
    str = str.substr(cut_at+1);
  }

  if(str.length() > 0)
    *oi++ = str;
  
  return;

}
#endif //SOCKET_TYPEDEFS_H