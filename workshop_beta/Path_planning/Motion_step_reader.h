#ifndef MOTION_STEP_READER_H
#define MOTION_STEP_READER_H

#include "Path_planning\Motion_step_base.h"
#include "Path_planning\Motion_step_translational.h"
#include "Path_planning\Motion_step_rotational.h"

template <typename K>
class Motion_step_reader
{
private:
  std::istream& is;
public:
  Motion_step_reader (std::istream& input_stream) : is(input_stream) 
  {}
  Motion_step_base<K>* operator() ()
  {
    char c;
    is >> c;
    
    Motion_step_base<K>* motion_step;
    CGAL_postcondition (c == 'T' || c == 'R' || c == 'S');
    if (c == 'T')
      motion_step = new Motion_step_translational<K> ();
    else if (c == 'R')
      motion_step = new Motion_step_rotational<K> ();
    else if (c == 'S')
      motion_step = new Motion_step_static<K> ();
    else
        CGAL_postcondition(false);

    motion_step->read(is);
    return motion_step;
  }
}; //Motion_step_reader
#endif //MOTION_STEP_READER_H