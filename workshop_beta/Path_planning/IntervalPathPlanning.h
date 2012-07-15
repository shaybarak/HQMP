#ifndef INTERVAL_PATH_PLANNING_H 
#define INTERVAL_PATH_PLANNING_H 

#include "Utils\Interval_utils\Interval.h"
#include "Utils\Interval_utils\IntervalSet.h"

#include "Path_planning\PathPlanningUtils.h"

//we consider intervals as parts of the projectional line i.e. 
//+oo is connected to -oo.
//thus to represent a path we need a source, target and midpoint
template <typename K, typename Interval_set, typename OutputIterator>
void plan_path_in_interval( const typename K::FT& s,
                            const typename K::FT& t,
                            const Interval_set& interval_set,
                            OutputIterator& oi)
{
  const typename Interval_set::Interval_list& interval_list(interval_set.get_intervals());
  
  ///////////////////////////////////////////////////////////////////////////////////
  //validate input
  ///////////////////////////////////////////////////////////////////////////////////
  CGAL_precondition( (interval_list.size() == 1) || (interval_list.size() == 2) );
  CGAL_precondition_code (
    if (interval_list.size() == 1)
    {
      CGAL_precondition ( interval_list.front().contains(s) && 
                          interval_list.front().contains(t));
    }
    else //(interval_set.get_intervals().size() == 2)
    {
      CGAL_precondition (interval_list.front().is_lower_bounded() == false);
      CGAL_precondition (interval_list.back().is_higher_bounded() == false);

      bool containes_s_front = interval_list.front().contains(s);
      bool containes_t_front = interval_list.front().contains(t);
      bool containes_s_back  = interval_list.back().contains(s);
      bool containes_t_back  = interval_list.back().contains(t);
      CGAL_precondition((containes_s_front  || containes_s_back) &&
                        (containes_t_front  || containes_t_back));
    }
    );

  ///////////////////////////////////////////////////////////////////////////////////
  CGAL::Comparison_result cr = CGAL::compare(s,t);
  if (cr == CGAL::EQUAL)
    return;
  
  //add source
  *oi++ = s;

  //add midpoint
  if (interval_list.size() == 1)
  {
    *oi++ = (s+ t) / 2; //midpoint
  }
  else //interval_list.size() ==2
  {
    typename Interval_set::Interval gap(interval_list.front().higher(),
                                        interval_list.back().lower());
    K::FT value_in_gap = gap.point_in_interval();
    if (cr == CGAL::SMALLER)  //s < t
    {
      if ((CGAL::compare(s,value_in_gap) == CGAL::SMALLER) &&
          (CGAL::compare(value_in_gap,t) == CGAL::SMALLER))
      {
        //...s...| ... v ... | ...t...
        *oi++ = (s-1); //midpoint
      }
      else
      {
        //...s..t.| ... v ... | ......   or
        //........| ... v ... | ...s..t. or
        *oi++ = (s+ t) / 2; //midpoint
      }
    }
    else //(cr == CGAL::LARGER) => s > t
      if ((CGAL::compare(t,value_in_gap) == CGAL::SMALLER) &&
          (CGAL::compare(value_in_gap,s) == CGAL::SMALLER))
      {
        //...t... | ... v ... | ...s...
        *oi++ = (s+1); //midpoint
      }
      else
      {
        //...s..t.| ... v ... | ......   or
        //........| ... v ... | ...s..t. or
        *oi++ = (s+ t) / 2; //midpoint
      }
  }


  //add target
  *oi++ = t;
  return;
}
#endif //INTERVAL_PATH_PLANNING_H 