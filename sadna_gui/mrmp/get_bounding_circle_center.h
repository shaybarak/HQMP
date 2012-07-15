#ifndef BOUNDING_CIRCLE_H
#define BOUNDING_CIRCLE_H

#include <CGAL/Cartesian.h>
#include <CGAL/Min_sphere_of_spheres_d.h>

//get center of bounding circles of set of points given by an iterator
//assumption: iterator type is std::pair<double, double>
template <typename InputIterator>
std::pair<double, double> get_bounding_circle_center(InputIterator begin, InputIterator end)
{  
	typedef double						FT;
	typedef CGAL::Cartesian<FT>			K;
	typedef K::Point_2					CPoint;
	typedef typename CGAL::Min_sphere_of_spheres_d_traits_2<K, FT>	B_circle_traits;
	typedef B_circle_traits::Sphere									Circle;
	typedef CGAL::Min_sphere_of_spheres_d<B_circle_traits>			B_circle;

	//translate each point into a circle 
	std::vector<Circle> circles;
	for (InputIterator iter = begin; iter != end; ++iter)
	{
		CPoint p(iter->first, iter->second);
		circles.push_back(Circle(p, 0)); //circle with center at point and no radius
	}

	//join all circles
	B_circle bounding_circle (circles.begin(),circles.end());
  
	//get center
	B_circle::Cartesian_const_iterator center_iterator (bounding_circle.center_cartesian_begin ());
	B_circle::Result x (*center_iterator);
	++center_iterator;
	B_circle::Result y (*center_iterator);

	//translate returned value to double
	return std::make_pair(x,y);
}
#endif //BOUNDING_CIRCLE_H