#ifndef BOUNDING_UTILS_H
#define BOUNDING_UTILS_H

#include <CGAL/Min_sphere_of_spheres_d.h>

//get center of bounding circles 
//of set of points given by an iterator
template <typename K, typename InputIterator>
typename K::Point_2 get_bounding_circle_center(InputIterator begin,
                                               InputIterator end)
{  
  typedef typename CGAL::Min_sphere_of_spheres_d_traits_2<K,typename K::FT>	B_circle_traits;
  typedef B_circle_traits::Sphere								            Circle;
  typedef CGAL::Min_sphere_of_spheres_d<B_circle_traits>		            B_circle;

  //translate each point into a circle 
  std::vector<Circle> circles;
  for (InputIterator iter = begin; iter != end; ++iter)
    circles.push_back(Circle(*iter,0)); //circle with center at point and no radius

  //join all circles
  B_circle bounding_circle (circles.begin(),circles.end());
  
  //get center
  B_circle::Cartesian_const_iterator center_iterator (bounding_circle.center_cartesian_begin ());
  B_circle::Result result_x (*center_iterator);
  ++center_iterator;
  B_circle::Result result_y (*center_iterator);

  //translate returned value according to NT used
  K::FT x,y;
  if (boost::is_same<B_circle::Result,K::FT>::value)
  {
    //inexact number type
#ifdef DOUBLE_USED
    x = result_x;
    y = result_y;
#endif //DOUBLE_USED
  }
  else
  {
#ifndef DOUBLE_USED
    //exact number type
    CGAL_precondition (result_x.second == 0);  //we assume center will be rational
    CGAL_precondition (result_y.second == 0);  //we assume center will be rational
    x = result_x.first;
	y = result_y.first;
#else
    assert(false);
#endif //!DOUBLE_USED
  }
  return K::Point_2(x,y);
}
#endif //BOUNDING_UTILS_H