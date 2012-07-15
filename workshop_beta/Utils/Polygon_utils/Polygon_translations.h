#ifndef POLYGON_TRANSLATIONS_H
#define POLYGON_TRANSLATIONS_H

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

/////////////////////////
//Translation functions//
/////////////////////////
template<typename K>
typename K::Point_2 translate_point (const typename K::Point_2& original,   //original point
                                     const typename K::Point_2& translation)//translation value
{
  K::FT x (translation.x() + original.x());
  K::FT y (translation.y() + original.y());
  K::Point_2 p(x,y);
  return p;
}

template<typename K>
void translate_polygon (const CGAL::Polygon_2<K>&  original,
                        CGAL::Polygon_2<K>&        translated,
                        const typename K::Point_2& translation)
{
  if (!translated.is_empty ())
		translated.clear ();
  
  typename CGAL::Polygon_2<K>::Vertex_const_iterator vit;
  for (vit = original.vertices_begin(); vit != original.vertices_end(); ++vit)
  {
    typename K::FT x (translation.x() + vit->x());
    typename K::FT y (translation.y() + vit->y());
    typename K::Point_2 p(x,y);
	translated.push_back(p);
    CGAL_postcondition (p == translate_point<K> (*vit,translation));
  }
  return;
}

template<typename K>
void translate_polygon (const CGAL::Polygon_with_holes_2<K>&  original,
                        CGAL::Polygon_with_holes_2<K>&        translated,
                        const typename K::Point_2&            translation)
{
  typedef typename CGAL::Polygon_2<K>                         Polygon;
  typedef typename CGAL::Polygon_with_holes_2<K>              Polygon_with_holes;

  Polygon outer;
  translate_polygon (original.outer_boundary(), outer, translation);

  std::vector <Polygon> holes;
  Polygon_with_holes::Hole_const_iterator hit;
  for (hit = original.holes_begin(); hit != original.holes_end(); ++hit)
  {
    Polygon tmp;
    translate_polygon<K> (*hit, tmp, translation);
    holes.push_back(tmp);
  }

  translated = Polygon_with_holes (outer, holes.begin(), holes.end() );

  return;
}

#endif  //POLYGON_TRANSLATIONS_H
