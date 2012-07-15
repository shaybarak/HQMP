#ifndef MOTION_STEP_WRITER_H
#define MOTION_STEP_WRITER_H

#include "Path_planning\Motion_step_base.h"
#include "Path_planning\Motion_step_translational.h"
#include "Path_planning\Motion_step_rotational.h"

template <typename K>
class Motion_step_writer
{
public:
  typedef Motion_step_base<K> Motion_step;
private:
  std::ostream& os;
public:
  Motion_step_writer (std::ostream& output_stream) : os(output_stream)
  {}
  void operator() (Motion_step* motion_step_base_ptr)
  {
    CGAL_precondition (motion_step_base_ptr->type()!= Motion_step::UNINITIALIZED);
    if (motion_step_base_ptr->type() == Motion_step::TRANSLATION)
      os << 'T'<< std::endl;
    else if (motion_step_base_ptr->type() == Motion_step::ROTATION)
      os << 'R'<< std::endl;
    else if (motion_step_base_ptr->type() == Motion_step::STATIC)
      os << 'S'<< std::endl;
    else
        CGAL_postcondition(false);

    motion_step_base_ptr->write(os);
  }
}; //Motion_step_writer
#endif //MOTION_STEP_WRITER_H