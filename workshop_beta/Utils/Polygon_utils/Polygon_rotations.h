#ifndef POLYGON_ROTATIONS_H
#define POLYGON_ROTATIONS_H

#include <CGAL/Polygon_2.h>
#include "Utils\Rotation_utils\Rotation.h"

//////////////////////
//Rotation functions//
//////////////////////
template<typename K>
typename K::Point_2 rotate_point (const typename K::Point_2& original,                            //original point
                                  const typename Rotation<typename K::FT> & rotation,             //rotation value
                                  const typename K::Point_2& origianl_reference= K::Point_2(0,0)) //reference point of the rotation
{
  typedef typename K::FT NT;
  NT sin (rotation.sin());
  NT cos (rotation.cos());

  NT a  (original.y() - origianl_reference.y());
  NT b  (original.x() - origianl_reference.x());

  NT a_ (a*cos + b*sin);
  NT b_ (b*cos - a*sin);

  NT x  (origianl_reference.x() + b_);
  NT y  (origianl_reference.y() + a_);

  return K::Point_2 (x,y);
}

template<typename K>
void rotate_point (const typename K::Point_2& original,                                     //original point
                   const typename Rotation<typename K::FT>& rotation,                       //rotation value
                   typename K::Point_2& rotated,                                            //rotated point
                   const typename K::Point_2& origianl_reference= typename K::Point_2(0,0)) //reference point of the rotation
{
  rotated = rotate_point <K> ( original,rotation,origianl_reference);
  return;  
}

template<typename K>
void rotate_polygon (const CGAL::Polygon_2<K>&  original,                             //original polygon
                     CGAL::Polygon_2<K>&        rotated,                              //rotated polygon
                     const Rotation<typename K::FT>&  rotation,                       //rotation value
                     const typename K::Point_2& original_reference= K::Point_2(0,0))  //reference point of the rotation
{
  typedef typename CGAL::Polygon_2<K>::Vertex_const_iterator  V_iterator;
  typedef typename K::Point_2                                 Point;
    
  if (rotation.has_complete_rotation())
  {
    rotated = original;
    return;
  }

  rotated.clear();
  for (V_iterator vit(original.vertices_begin()) ; vit != original.vertices_end(); ++vit)
  {
    Point rotated_point (rotate_point<K> (*vit, rotation, original_reference));
    rotated.push_back(rotated_point);
  }

  return;
}

#endif  //POLYGON_ROTATIONS_H
