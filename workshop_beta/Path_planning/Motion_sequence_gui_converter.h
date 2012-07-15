#ifndef MOTION_SEQUENCE_WRITER_FOR_GUI_H
#define MOTION_SEQUENCE_WRITER_FOR_GUI_H

#include "Path_planning\Motion_step_base.h"
#include "Path_planning\Motion_step_translational.h"
#include "Path_planning\Motion_step_rotational.h"


template <typename K>
class Motion_sequence_gui_converter
{
public:
  typedef typename Motion_sequence<K>  Motion_sequence;
  typedef typename K::Point_2          Point;
  typedef typename Reference_point<K>  Reference_point;
public:
class Gui_ref_point_3d
  {
  public:
    Gui_ref_point_3d(double x_=0, double y_=0, double t_=0)
      :x(x_), y(y_), t(t_) {}
  public:
    double x, y, t;
  }; //Gui_ref_point_3d  
  
  typedef std::vector <Gui_ref_point_3d> Path_3d;
public:
  template <typename OutputIterator>
  void operator() (Motion_sequence& motion_sequence, OutputIterator& oi)
  {
    const Motion_sequence::Motion_vec& motion_vec(motion_sequence.get_sequence());

    if (motion_vec.empty())
      return ;    
    
    //insert source
    *oi++ = convert(motion_vec.front()->source());

    //go over every motion step; if needed, refine, if not compute difference from last location
    typename Motion_sequence::MS_base_ptr ms_ptr;
    BOOST_FOREACH (ms_ptr, motion_vec)
    {
      if (to_refine(ms_ptr))
      {
        std::pair<Gui_ref_point_3d, Gui_ref_point_3d> refined_step;
        refine_step(ms_ptr, refined_step);
        *oi++ = refined_step.first;
        *oi++ = refined_step.second;
      }
      else //no refinement need
        *oi++ = difference(ms_ptr->source(), ms_ptr->target()); 
    }
    return;
  }
private:
  bool to_refine(typename Motion_sequence::MS_base_ptr ms_ptr)
  {
    if (ms_ptr->type() != Motion_sequence::MS_base::ROTATION)
      return false;

    if ((static_cast<typename Motion_sequence::MS_rotational*> (ms_ptr))->get_angular_difference_in_deg() <180)
      return false;
    
    return true;
  }
  void refine_step( typename Motion_sequence::MS_base_ptr ms_ptr, 
                    std::pair<Gui_ref_point_3d, Gui_ref_point_3d>& refined_step)
  {
    CGAL_precondition (ms_ptr->type() == Motion_sequence::MS_base::ROTATION);
    
    Motion_sequence::MS_rotational* ms_rotational_ptr =  static_cast<typename Motion_sequence::MS_rotational*> (ms_ptr);
    double diff = ms_rotational_ptr->get_angular_difference_in_deg();
    double delta_t;
    if (ms_rotational_ptr->orientation() == CGAL::COUNTERCLOCKWISE)
      delta_t = diff/2;
    else //(ms_rotational_ptr->orientation() == CGAL::CLOCKWISE)
      delta_t = -diff/2;

    CGAL_precondition (ms_ptr->source().get_location() == ms_ptr->target().get_location());
    refined_step.first = Gui_ref_point_3d(0, //to change in x 
                                          0, //to change in y
                                          delta_t);
    refined_step.second = Gui_ref_point_3d(0, //to change in x 
                                           0, //to change in y
                                           delta_t);
    return;
  }
  Gui_ref_point_3d convert (const Reference_point& r)
  {
    double x = CGAL::to_double(r.get_location().x());
    double y = CGAL::to_double(r.get_location().y());
    double t = r.get_rotation().to_angle();
    return Gui_ref_point_3d(x, y, t);
  }
  Gui_ref_point_3d difference (const Reference_point& s, const Reference_point& t)
  {
    Gui_ref_point_3d s_ = convert(s);
    Gui_ref_point_3d t_ = convert(t);
    return Gui_ref_point_3d(t_.x - s_.x, 
                            t_.y - s_.y, 
                            t_.t - s_.t);
  }


}; //Motion_sequence_writer_for_gui
#endif //MOTION_SEQUENCE_WRITER_FOR_GUI_H