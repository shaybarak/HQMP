#ifndef POLYGON_IO_T_H
#define POLYGON_IO_T_H

#include <iostream>  
#include <fstream>   

template<typename K>
void print_polygon(const CGAL::Polygon_2<K>& polygon)
{
  typedef typename CGAL::Polygon_2<K>::Vertex_const_iterator  V_iterator;
  
  std::cout <<"Polygon vertices: " << std::endl;
  for (V_iterator vi (polygon.vertices_begin ()); vi!= polygon.vertices_end (); ++vi)
  {
    std::cout << "(" << vi->x() << "  ,  " << vi->y() << ")" << std::endl; 
  }
  return;
}

template<typename K>
void print_polygon(const CGAL::Polygon_with_holes_2<K>& polygon)
{
  std::cout <<"Polygon outer boundary: " << std::endl;
  print_polygon<K> (CGAL::Polygon_2<K>(polygon.outer_boundary ()));

  std::cout <<"Polygon holes: " << std::endl;
  for (CGAL::Polygon_with_holes_2<K>::Hole_const_iterator hi = polygon.holes_begin  (); hi!= polygon.holes_end  (); ++hi)
  {
    print_polygon<K> (CGAL::Polygon_2<K>(*hi));
  }
  return;
}

template<typename K>
void print_polygon_vector (const std::vector<CGAL::Polygon_2<K> >& polygons)
{
  typedef std::vector<CGAL::Polygon_2<K> >::const_iterator Iter;
  for (Iter iter (polygons.begin()); iter != polygons.end(); ++iter)
  {
    print_polygon<K>(*iter);
    std::cout << std::endl;
  }
  return;
}

template<typename K>
void write_polygon_to_file (const CGAL::Polygon_2<K>& polygon, const std::string& file_name)
{
  ofstream file;
  file.open(file_name.c_str());

  file << polygon.size() << std::endl;
  for (int i(0); i < polygon.size() ; ++i)
  {
    file << CGAL::to_double (polygon[i].x());
    file << " ";
    file << CGAL::to_double (polygon[i].y());
    file << std::endl;
  }

  file.close();
}

#endif //POLYGON_IO_T_H