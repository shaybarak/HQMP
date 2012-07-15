#ifndef LESS_THAN_POINT_H
#define LESS_THAN_POINT_H

template <typename K>
struct Less_than_point_2
{
  bool operator()(const typename K::Point_2 p1, 
                  const typename K::Point_2 p2) const 
  {
    return (( CGAL::compare_xy(p1,p2) == CGAL::SMALLER ) ?
            true:
            false);
  }
};

template <typename K>
struct Less_than_point_3
{
  bool operator()(const typename K::Point_3 p1, 
                  const typename K::Point_3 p2) const 
  {
    return (( CGAL::compare_xyz(p1,p2) == CGAL::SMALLER ) ?
            true:
            false);
  }
};

#endif //LESS_THAN_POINT_H