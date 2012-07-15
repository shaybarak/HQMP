#ifndef FSC_PATH_PLANNING_H
#define FSC_PATH_PLANNING_H

#include "Utils\Rotation_utils\Rotation.h"
#include "Utils\ReferencePoint.h"
#include "FSC.h"
#include "Manifolds\MMSTypedefs.h"
#include "Manifolds\Fixed_angle\Fixed_angle_constraint.h"
#include "Manifolds\Fixed_angle\Fixed_angle_fsc.h"  
#include "Manifolds\Fixed_point\Fixed_point_constraint.h"
#include "Manifolds\Fixed_point\Fixed_point_fsc.h"  

#include "Path_planning\PathPlanningUtils.h"
#include "Path_planning\PolygonPathPlanning.h"
#include "Path_planning\IntervalPathPlanning.h"

#include "Path_planning\Motion_step_translational.h"
#include "Path_planning\Motion_step_rotational.h"
#include "Path_planning\Motion_sequence.h"

namespace mms{

template <typename Fsc>
void plan_path(const Fsc* fsc_ptr, 
               const typename Fsc::Ref_p& source, 
               const typename Fsc::Ref_p& target,
               Motion_sequence<typename Fsc::K>& motion_sequence)
{
  switch (fsc_ptr->get_constraint_type())
  {
  case FIXED_ANGLE:
    return plan_path___fixed_angle(fsc_ptr, source, target, motion_sequence);
  case FIXED_POINT:
    return plan_path__fixed_point(fsc_ptr, source, target, motion_sequence);
  default:
    assert(false);
  } 
}

template <typename Fsc>
void plan_path___fixed_angle (const Fsc* fsc_ptr,
                              const typename Fsc::Ref_p& source, 
                              const typename Fsc::Ref_p& target,
                              Motion_sequence<typename Fsc::K>& motion_sequence)
{
  typedef typename Fsc::K           K;
  typedef typename Fsc::Ref_p       Ref_p;
  typedef typename Ref_p::Point     Point;
  typedef typename Ref_p::Rotation  Rot;
  
  CGAL_precondition(fsc_ptr->get_constraint_type() == FIXED_ANGLE);
  CGAL_precondition(fsc_ptr->contains(source));
  CGAL_precondition(fsc_ptr->contains(target));
  
  Rot fsc_rotation = fsc_ptr->get_free_space_constraint<Fixed_angle_constraint<K> >().restriction();
  
  CGAL_precondition(fsc_rotation == source.get_rotation());
  CGAL_precondition(fsc_rotation == target.get_rotation());
  
  //compute path in polygon
  std::list<Point> point_path;
  plan_path_in_polygon<K>(source.get_location(),
                          target.get_location(),
                          fsc_ptr->get_free_space_feature<Fixed_angle_fsc<K> >().cell().polygon(),
                          point_path);

  //convert point path to a sequence of Motion_step_translational
  std::list<Point>::iterator curr = point_path.begin();
  std::list<Point>::iterator next = point_path.begin();
  ++next;

  while (next != point_path.end())
  {
    Motion_step_translational<K>* motion_step_ptr = new Motion_step_translational<K>(*curr, *next, fsc_rotation);
    motion_sequence.add_motion_step(motion_step_ptr);
    ++curr;
    ++next;
  }
  
  return;
}

template <typename Fsc>
void plan_path__fixed_point ( const Fsc* fsc_ptr,
                              const typename Fsc::Ref_p& source, 
                              const typename Fsc::Ref_p& target,
                              Motion_sequence<typename Fsc::K>& motion_sequence)
{
  CGAL_precondition(fsc_ptr->get_constraint_type() == FIXED_POINT);
  CGAL_precondition(fsc_ptr->contains(source));
  CGAL_precondition(fsc_ptr->contains(target));
  
  if (source == target)
      return; //nothing to do
  
  typedef typename Fsc::K           K;
  typedef typename Fsc::Fsc_fp      Fsc_fp;
  typename K::Point_2 fsc_point = fsc_ptr->get_free_space_constraint<Fixed_point_constraint<K> >().restriction();
  
  //compute path in polygon
  std::vector<typename K::FT> tau_path;
  plan_path_in_interval<K>( FixedPoint::get_parametrization_theta<K>(source.get_rotation()),
                            FixedPoint::get_parametrization_theta<K>(target.get_rotation()),
                            fsc_ptr->get_free_space_feature<Fsc_fp>().cell(),
                            std::back_inserter(tau_path));
  
  CGAL_postcondition(tau_path.size() == 3); //source, target, mid
  
  //convert point path to Motion_step_rotational
  CGAL::Orientation orientation = get_orientation(tau_path[0], tau_path[1], tau_path[2]);  
  Motion_step_rotational<K>* motion_step_ptr = new Motion_step_rotational<K>(fsc_point, 
                                                                             source.get_rotation(), target.get_rotation(), 
                                                                             orientation);
  motion_sequence.add_motion_step(motion_step_ptr);

  return;
}

} //namespace mms{
#endif //FSC_PATH_PLANNING_H