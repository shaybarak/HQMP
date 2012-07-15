#ifndef TARGET_CONFIGURATIONS_MANAGER_H
#define TARGET_CONFIGURATIONS_MANAGER_H

#include "Path_planning\Motion_step_base.h"
#include "Path_planning\Motion_step_translational.h"
#include "Path_planning\Motion_step_rotational.h"

template <typename K>
class Target_configurations_manager
{
public:
  typedef Motion_step_base<K>               MS_base;
  typedef Motion_step_translational<K>      MS_translational;
  typedef Motion_step_rotational<K>         MS_rotational;

  typedef Reference_point<K>                Reference_point;
  typedef std::vector<Reference_point>      Reference_point_vec;
  
  enum Status { N = 0, 
                A = 1, 
                B = 2, 
                AB = 3,
                Z  = 4};
  typedef std::vector<Status>               Status_vec;
private:
  Reference_point_vec   reference_points_values;
  Status_vec            reference_points_status;
  unsigned int          a_counter;
  unsigned int          b_counter;
public:
  Target_configurations_manager(Reference_point_vec& initial_target_configurations,
                                Reference_point_vec& additional_target_configurations)
    : a_counter(0), b_counter(0)
  {
    BOOST_FOREACH(Reference_point r, initial_target_configurations)
    {
      reference_points_values.push_back(r);
      reference_points_status.push_back(N);
    }
    BOOST_FOREACH(Reference_point r, additional_target_configurations)
    {
      reference_points_values.push_back(r);
      reference_points_status.push_back(Z);
    }
  }
  void activate_additional_target_configurations()
  {
    for (unsigned int i(0); i<reference_points_status.size(); ++i)
      if (reference_points_status[i] == Z)
        reference_points_status[i] = N;
    return;
  }
  bool mark_contained_configurations( MS_base* motion_step_ptr, bool is_player_a)
  {
    CGAL_precondition ( (motion_step_ptr->type() == MS_base::TRANSLATION) ||
                        (motion_step_ptr->type() == MS_base::ROTATION) );

    if (motion_step_ptr->type() == MS_base::TRANSLATION)
      mark_contained_configurations(static_cast<MS_translational*> (motion_step_ptr), is_player_a);
    else // (motion_step_ptr->type() == Motion_step::ROTATION)
      mark_contained_configurations(static_cast<MS_rotational*> (motion_step_ptr), is_player_a);
   
    return completed_task(is_player_a);
  }
  bool completed_task(bool is_player_a)
  {
    if (is_player_a)
      return (a_counter == reference_points_status.size());
    else // !is_player_a
      return (b_counter == reference_points_status.size());
  }

  int get_score(bool is_player_a)
  {
    return (is_player_a ? a_counter : b_counter);
  }
  void get_configuration_status(std::vector<int>& status_vec)
  {
    CGAL_precondition (status_vec.empty());
    status_vec.reserve (reference_points_status.size());
    BOOST_FOREACH(Status status, reference_points_status)
      status_vec.push_back(static_cast<int> (status));
    return;
  }
private:
  void mark_contained_configurations( MS_translational* motion_step_ptr, bool is_player_a)
  {
    typename K::FT sx = motion_step_ptr->source().get_location().x();
    typename K::FT tx = motion_step_ptr->target().get_location().x();
    typename K::FT sy = motion_step_ptr->source().get_location().y();
    typename K::FT ty = motion_step_ptr->target().get_location().y();

    for (unsigned int i(0); i< reference_points_values.size(); ++i)
    {
      if (reference_points_status[i] == Z)
        continue; //inactive
      
      if (motion_step_ptr->source().get_rotation() != reference_points_values[i].get_rotation())
        continue;

      //same angle, check if between source and target.
      typename K::FT r;
      typename K::FT x = reference_points_values[i].get_location().x();
      typename K::FT y = reference_points_values[i].get_location().y();

      if (tx != sx)
        r = (tx - x) / (tx - sx);
      else //(ty!=sy)
        r = (ty - y) / (ty - sy);

      if ( r>=0 && r <=1)
      {
        //found a hit, check if it is new
        if (is_index_marked(i, is_player_a) == false)
          mark_index(i, is_player_a);        
      }       
    }
  }
  void mark_contained_configurations( MS_rotational* motion_step_ptr, bool is_player_a)
  {
    CGAL::Orientation o = motion_step_ptr->orientation();
    typename MS_rotational::Rotation s = motion_step_ptr->source().get_rotation();
    typename MS_rotational::Rotation t = motion_step_ptr->target().get_rotation();

    Rotation_range_absolute<typename K::FT> range(o == CGAL::COUNTERCLOCKWISE ? s : t,
                                                  o == CGAL::COUNTERCLOCKWISE ? t : s);
    

    for (unsigned int i(0); i< reference_points_values.size(); ++i)
    {
      if (motion_step_ptr->source().get_location() != reference_points_values[i].get_location())
        continue;

      //same location, check if between source and target.
      typename MS_rotational::Rotation r = reference_points_values[i].get_rotation();
      if ((r == s) || (r == t) || 
          (range.is_in_range(r)) )
      {
        //found a hit, check if it is new
        if (is_index_marked(i, is_player_a) == false)
          mark_index(i, is_player_a);        
      }       
    }
    return;
  }
  bool is_index_marked(unsigned int i, bool is_player_a)
  {
    CGAL_precondition(i >=0 && i < reference_points_status.size());
    
    Status status = reference_points_status[i];
    
    if (status == AB)
      return true;
    if (status == N)
      return false;

    return (is_player_a ? status == A :status == B);
  }
  void mark_index(unsigned int i, bool is_player_a)
  {
    CGAL_precondition(i >=0 && i < reference_points_status.size());
    switch (reference_points_status[i])
    {
    case (AB) : 
      return;
    case (N) :
      {
        reference_points_status[i] = (is_player_a ? A : B);
        is_player_a ? a_counter++ : b_counter++;
        return;
      }
    case (A) :
      {
        if (is_player_a)
          return;
        else //!is_player_a
        {
          reference_points_status[i] = AB;
          b_counter++;
          return;
        }
      }
    case (B) :
      {
        if (!is_player_a)
          return;
        else //is_player_a
        {
          reference_points_status[i] = AB;
          a_counter++;
          return;
        }
      }
    }
    return;
  }

}; //Target_configurations_manager
#endif //TARGET_CONFIGURATIONS_MANAGER_H