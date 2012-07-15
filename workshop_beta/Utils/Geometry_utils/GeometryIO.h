#ifndef GEOMETRY_IO_T_H
#define GEOMETRY_IO_T_H

template <typename K> 
void print_point_nice(const typename K::Point_2& p)
{
  cout  << "("
        << CGAL::to_double(p.x())
        << " , "
        << CGAL::to_double(p.y())
        << ")";
  return;
}

template <typename K> 
void print_edge_nice(const typename K::Segment_2& e)
{
  cout << "[ ";
  print_point_nice<K>(e.source());
  cout << " , ";
  print_point_nice<K>(e.target());
  cout << " ]";
  return;
}


#endif //GEOMETRY_IO_T_H