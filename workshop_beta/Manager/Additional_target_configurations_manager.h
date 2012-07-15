#ifndef ADDITIONAL_TARGET_MANAGER_H
#define ADDITIONAL_TARGET_MANAGER_H

#include "Manager\Time_frame.h"

class Additional_target_configurations_manager
{
private:
  int                             interval_to_add_target_configurations;
  Time_frame*                     time_frame_ptr;
              
public:
  Additional_target_configurations_manager (int interval_to_add_target_configurations_,
                                            Time_frame*   time_frame_ptr_) 
    : interval_to_add_target_configurations(interval_to_add_target_configurations_),
      time_frame_ptr(time_frame_ptr_)
  {}

  bool added_target_configurations()
  {
    return (time_frame_ptr->id() >= interval_to_add_target_configurations);
  }

}; //Additional_target_configurations_manager
#endif //ADDITIONAL_TARGET_MANAGER_H