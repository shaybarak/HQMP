#ifndef MOTION_SEQUENCE_H_
#define MOTION_SEQUENCE_H_

#include "Utils\ReferencePoint.h"
#include "Path_planning\Motion_step_base.h"
#include "Path_planning\Motion_step_translational.h"
#include "Path_planning\Motion_step_rotational.h"
#include "Path_planning\Motion_step_static.h"

#include "Path_planning\Motion_step_writer.h"
#include "Path_planning\Motion_step_reader.h"

template <typename K>
class Motion_sequence
{
public:
  typedef Motion_step_base<K>             MS_base;
  typedef Motion_sequence<K>              Self;
  typedef Motion_step_translational<K>    MS_translational;
  typedef Motion_step_rotational<K>       MS_rotational;
  typedef Motion_step_static<K>			      MS_static;
public:
  typedef MS_base*                          MS_base_ptr;
  typedef std::vector<typename MS_base_ptr> Motion_vec;
private:
  Motion_vec motion_sequence;
public:
  Motion_sequence() {}
  ~Motion_sequence() 
  {
      clear();
  }
  void clear()
  {
      BOOST_FOREACH(MS_base_ptr motion_step_ptr, motion_sequence)
          delete motion_step_ptr;
      motion_sequence.clear();
  }
  template <typename InputIterator>
  Motion_sequence(InputIterator& begin, InputIterator& end) 
  {
    for (InputIterator iter = begin; iter != end; ++iter)
      add_motion_step(*iter);    
  }
  void add_motion_step(const MS_base_ptr& motion_step)
  {
    CGAL_precondition ( (motion_sequence.empty()) || 
                        (motion_sequence.back()->target() == motion_step->source()) );
    
    motion_sequence.push_back(motion_step);
    return;
  }
  void add_motion_sequence(const Self& other_motion_sequence)
  {
    const Motion_vec& other_motion_sequence_vec = other_motion_sequence.get_sequence();
    BOOST_FOREACH(MS_base_ptr motion_step_ptr, other_motion_sequence_vec)
    {
        //ptr needs to be copied as each Motion_sequence class is in charge of deleting it's own MS_base_ptr
        CGAL_precondition (motion_step_ptr->type() != MS_base::UNINITIALIZED);
        switch (motion_step_ptr->type())
        {
        case (MS_base::TRANSLATION) : 
            {
                MS_translational* motion_step_ptr_copy = new MS_translational(* static_cast<MS_translational*> (motion_step_ptr));
                add_motion_step(motion_step_ptr_copy);
                break;
            }
        case (MS_base::ROTATION):
            {
                MS_rotational* motion_step_ptr_copy = new MS_rotational(* static_cast<MS_rotational*> (motion_step_ptr));
                add_motion_step(motion_step_ptr_copy);
                break;
            }
        case (MS_base::STATIC):
            {
                MS_static* motion_step_ptr_copy = new MS_static(* static_cast<MS_static*> (motion_step_ptr));
                add_motion_step(motion_step_ptr_copy);
                break;
            }
        }
    }
    return;
  }
  const Motion_vec& get_sequence() const
  {
    return motion_sequence;
  }
  void reverse_motion_sequence()
  {
      BOOST_FOREACH (MS_base_ptr motion_step, motion_sequence)
        motion_step->reverse_motion_step();
   
      std::reverse(motion_sequence.begin(), motion_sequence.end());
  }

public:
  double motion_time( const double translational_speed, //unit per second
                      const double rotational_speed) //full rotation per second
  {
    double t(0);
    BOOST_FOREACH(MS_base_ptr motion_step_ptr, motion_sequence)
    {
      if (motion_step_ptr->type() == MS_base::TRANSLATION)
        t+=motion_step_ptr->approximate_motion_time(translational_speed);
      if (motion_step_ptr->type() == MS_base::ROTATION)
        t+=motion_step_ptr->approximate_motion_time(rotational_speed);
    }
    return t;      
  }
public:
  void read(std::istream& is)
  {
    int n;    
    is >> n;

    Motion_step_reader<K> reader(is);
    for (int i(0); i<n; ++i)
      add_motion_step(reader());
    
    return;
  }
  void write(std::ostream& os)
  {
    if (motion_sequence.empty())
      return;

    os << motion_sequence.size() <<std::endl;
    Motion_step_writer<K> writer(os);
    BOOST_FOREACH(MS_base_ptr motion_step_ptr, motion_sequence)
      writer(motion_step_ptr);

    return;
  }
}; //Motion_sequence
#endif //MOTION_SEQUENCE_H_