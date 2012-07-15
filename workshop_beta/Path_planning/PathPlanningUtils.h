#ifndef PATH_PLANING_UTILS
#define PATH_PLANING_UTILS

#include "Path_planning\Motion_step_base.h"
#include "Path_planning\Motion_sequence.h"

template <typename FT>
CGAL::Orientation get_orientation(FT& s, FT& m, FT& t)
{
  CGAL::Orientation orientation;
  if (  ((s < t) && (s < m) && (m < t)) || //e.g. 30 60 90
        ((s > t) && (s < m) && (m > t)) || //e.g. 90 120 30
        ((s > t) && (s > m) && (m < t)) )  //e.g. 90 0 30
        orientation = CGAL::COUNTERCLOCKWISE;
  else
        orientation = CGAL::CLOCKWISE;

  return orientation;
}

template <typename K,typename OutputIterator>
void sample_rotations(const Rotation<typename K::FT>& s, 
                      const Rotation<typename K::FT>& t, 
                      CGAL::Orientation orientation,
                      OutputIterator& oi,
                      int num_of_samples = 5)
{
  double s_deg = s.to_angle();
  double t_deg = t.to_angle();
  int sign = ((orientation == CGAL::CLOCKWISE) ? -1 : +1);

  double n(0);
  double step (1.0 / double (num_of_samples));

  while (n<1)
  {
    double deg = s_deg + sign * n * (t_deg - s_deg);
    if (deg > 360)
      deg-=360;

    Rotation<typename K::FT> r = to_rotation<typename K::FT>(deg, DEG);

    *oi++ = r;
    n+=step;
  }
}

template <typename K,typename OutputIterator>
void sample_interval_inf( const typename K::FT& s, 
                          const typename K::FT& t, 
                          OutputIterator& oi,
                          double min_step = 0.0000001*INFINITY)
{
  if ((CGAL::abs(s) == INFINITY) &&
      (CGAL::abs(t) == INFINITY))
  {
    int sign = (s == INFINITY) ? 1 : -1;
    sample_interval_inf<K>(s, sign*1, oi);
    sample_interval<K>(sign*1, sign*-1, oi);
    sample_interval_inf<K>(sign*-1, t, oi);
    return;
  }

  if ((CGAL::sign(s) != CGAL::sign(t)) ||
      (CGAL::is_zero(s) || (CGAL::is_zero(t))))
  {
    if (CGAL::abs(s) == INFINITY)
    {
      int sign = (s == INFINITY) ? 1 : -1;
      sample_interval_inf<K>(s, sign*1, oi);
      sample_interval<K>(sign*1, t, oi);
    }
    else
    {
      CGAL_precondition(CGAL::abs(t) == INFINITY);
      {
        int sign = (t == INFINITY) ? 1 : -1;
        sample_interval<K>(s, sign*1, oi);
        sample_interval_inf<K>(sign*1, t, oi);
      }
    }
    return;
  }

  //here only one of s and t is infinity 
  //they share the same sign and the other is different from zero
  if (CGAL::abs(s) == INFINITY)
  {
    typename K::FT tmp(s);
    while (CGAL::abs(t) < CGAL::abs(tmp))
    {
      *oi++ = tmp;
      tmp = tmp/2;
    }
  }
  else // (CGAL::abs(t) == INFINITY)
  {
    typename K::FT tmp(s);
    while (CGAL::abs(t) > CGAL::abs(tmp))
    {
      *oi++ = tmp;
      tmp = tmp*2;
    }
  }
  return;
}


template <typename K,typename OutputIterator>
void sample_interval (const typename K::FT& s, 
                      const typename K::FT& t, 
                      OutputIterator& oi,
                      int num_of_samples = 5)
{
  if ((CGAL::abs(s) == INFINITY) ||
      (CGAL::abs(t) == INFINITY))
      return sample_interval_inf<K>(s, t, oi);

  CGAL_precondition (s != t);
  K::FT n(0);
  K::FT step (1.0 / double (num_of_samples));

  while (n<1)
  {
    *oi++ = s + n * (t - s);
    n+=step;
  }
  return;
}


template <typename K,typename OutputIterator>
void sample_segment (const typename K::Point_2& source, 
                     const typename K::Point_2& target, 
                     OutputIterator& oi,
                     int num_of_samples = 5)
{
  CGAL_precondition (!(source == target));
  K::FT n ;
  K::FT step (1.0 / double (num_of_samples));

  K::FT sx (source.x());
  K::FT sy (source.y());
  K::FT tx (target.x());
  K::FT ty (target.y());

  for (n = 0; n < 1; n +=step)
  {
    K::FT x,y;
    if ( CGAL::compare (n + step , 1 ) == CGAL::SMALLER)
    {
      x = sx + n * (tx - sx);
      y = sy + n * (ty - sy);
    }
    else
    {
      x = tx ;
      y = ty ;
    }
    *oi++ = (K::Point_2(x,y));
  }
  return;
}



template <typename K, typename OutputIterator>
void sample_motion_step_translation(Motion_step_base<K>* motion_step_ptr,
                                    OutputIterator& oi)
{
  typedef typename K::Point_2           Point_2;
  typedef Rotation<typename K::FT>      Rotation;
  typedef typename Reference_point<K>   Ref_p;
  
  CGAL_precondition (motion_step_ptr->type() == Motion_step_base<K>::TRANSLATION);
  Point_2 s = motion_step_ptr->source().get_location();
  Point_2 t = motion_step_ptr->target().get_location();
  Rotation r = motion_step_ptr->source().get_rotation();
  
  std::vector<Point_2> point_vec;
  sample_segment<K> (s, t, std::back_inserter(point_vec));

  BOOST_FOREACH(Point_2 p, point_vec)
    *oi++ = Ref_p(p, r);
  return;
}

template <typename K, typename OutputIterator>
void sample_motion_step_rotation( Motion_step_base<K>* motion_step_ptr,
                                  OutputIterator& oi)
{
  typedef typename K::Point_2           Point_2;
  typedef Rotation<typename K::FT>      Rotation;
  typedef typename Reference_point<K>   Ref_p;

  CGAL_precondition (motion_step_ptr->type() == Motion_step_base<K>::ROTATION);

  Point_2 p = motion_step_ptr->source().get_location();
  Rotation r_s = motion_step_ptr->source().get_rotation();
  Rotation r_t = motion_step_ptr->source().get_rotation();
  
  std::vector<Rotation> rotation_vec;
  sample_rotations<K> (r_s, r_t, static_cast<Motion_step_rotational<K>* >(motion_step_ptr)->orientation(), std::back_inserter(rotation_vec));

  BOOST_FOREACH(Rotation r, rotation_vec)
    *oi++ = Ref_p(p, r);
}


template <typename K, typename OutputIterator>
void sample_motion_sequence ( Motion_sequence<K>& motion_sequence,
                              OutputIterator& oi)
{
  const Motion_sequence<K>::Motion_vec& motion_vec = motion_sequence.get_sequence();
  BOOST_FOREACH(Motion_sequence<K>::MS_base_ptr motion_step_ptr, motion_vec)
  {
    if (motion_step_ptr->type() == Motion_step_base<K>::TRANSLATION)
      sample_motion_step_translation<K>(motion_step_ptr, oi);
  else
      sample_motion_step_rotation<K>(motion_step_ptr, oi);
  }
}

#endif // PATH_PLANING_UTILS