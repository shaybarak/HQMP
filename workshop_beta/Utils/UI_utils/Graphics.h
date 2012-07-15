#ifndef Graphics_H
#define Graphics_H

#include <QApplication>
#include <QtGui>
#include <CGAL/Qt/GraphicsViewNavigation.h>
#include <QLineF>
#include <QRectF>
#include <QGraphicsLineItem>

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polygon_set_2.h>


#include "Utils\ReferencePoint.h"
#include "Utils\Polygon_utils\ExtendedPolygon.h"


const int GRAPHICS_INIFINITY = 1000;
const QColor RED   (255,0,0);
const QColor GREEN (0,255,0);
const QColor BLUE  (0,0,255);
const QColor ORANGE(255,165,0);
const QColor LIGHT_BLUE(125,158,192);
const QColor GRAY  (127,127,127);
const QColor BLACK (0,0,0);

class Graphics
{
public:
  typedef QGraphicsLineItem*        Line_handle;
  typedef std::vector< Line_handle> Object_handle;
private:
	QApplication    app;
	QGraphicsScene  _scene;
    QGraphicsScene  _file_scene;
 	
	double _min_x;
	double _max_x;
	double _min_y;
	double _max_y;
	double _ratio_x;
	double _ratio_y;
public:
  //constructors
  Graphics( int argc, char* argv[],
            double min_x = -1, double max_x = 1, 
            double min_y = -1, double max_y = 1);
  //destructor
  ~Graphics(void);
  //displays current scene
  void display();
  //exports current scene
  void save_scene(const std::string& path_name);
  void save_display(const std::string& file_name);

  //clear the scene
  void clear();
  //edge drawing functions
  template <typename K> 
  void draw_edge (const typename K::Point_2& p1, const typename K::Point_2& p2,const QColor& color)
  {
    _scene.addLine(QLineF(  to_screen_double_x<K::FT> (p1.x()),
                            to_screen_double_y<K::FT> (p1.y()),
                            to_screen_double_x<K::FT> (p2.x()),
                            to_screen_double_y<K::FT> (p2.y())),
                   QPen(color));
    _file_scene.addLine(QLineF( to_file_double_x<K::FT> (p1.x()),
                                to_file_double_y<K::FT> (p1.y()),
                                to_file_double_x<K::FT> (p2.x()),
                                to_file_double_y<K::FT> (p2.y())),
                        QPen(color));
  }
  template <typename K> void draw_edge (const typename K::Segment_2& s,const QColor& color)
  {
    return (this->draw_edge<K> (s.source(), s.target(), color));
  }

  //cross drawing functions
  template <typename K> void draw_cross(const typename K::Point_2& p, const QColor& color, double size = CROSS_SIZE)
  {
    draw_edge<K>( K::Point_2(p.x()-size,p.y()),
                  K::Point_2(p.x()+size,p.y()),
                  color);
	draw_edge<K>( K::Point_2(p.x(),p.y()+size),
                  K::Point_2(p.x(),p.y()-size),
                  color);
	return;
  }
  //polgon drawing functions
  template <typename K> 
  void draw_polygon(const CGAL::Polygon_2<K>& polygon, const QColor& color)
  {
    CGAL::Polygon_2<K>::Edge_const_iterator ei;

    for (ei = polygon.edges_begin(); ei != polygon.edges_end(); ++ei)
      draw_edge<K>(*ei,color);
    
    return;
  }
  template <typename K> 
  void draw_polygon(const CGAL::Polygon_with_holes_2<K>& polygon, const QColor& color)
  {
    //outer boundary
	if (! polygon.is_unbounded())
      draw_polygon<K> (polygon.outer_boundary(),color);

	//holes
	CGAL::Polygon_with_holes_2<K>::Hole_const_iterator hit;
	for (hit = polygon.holes_begin(); hit != polygon.holes_end(); ++hit)
      draw_polygon<K>(*hit,color);
  }
  template <typename K> 
  void draw_polygons(const CGAL::Polygon_set_2<K>& workspace, const QColor& color)
  {
    std::list<CGAL::Polygon_with_holes_2<K> > res;
	std::list<CGAL::Polygon_with_holes_2<K> >::const_iterator pi;
	workspace.polygons_with_holes (std::back_inserter (res));

	for (pi = res.begin(); pi != res.end(); ++pi)
      draw_polygon<K>(*pi,color);

    return;
  }
  template <typename K> 
  void draw_polygons(const std::vector <CGAL::Polygon_2<K> >& workspace, const QColor& color)
  {
    std::vector <CGAL::Polygon_2<K> >::const_iterator pi;
	for (pi=workspace.begin(); pi != workspace.end(); ++pi)
		draw_polygon<K>(*pi,color);

    return;
  }
  template <typename K> 
  void draw_polygons(const std::vector <CGAL::Polygon_with_holes_2<K> >& workspace, const QColor& color)
  {
    std::vector <CGAL::Polygon_with_holes_2<K> >::const_iterator pi;
	for (pi=workspace.begin(); pi != workspace.end(); ++pi)
		draw_polygon<K>(*pi,color);

    return;
  }

  //utils
  template <typename K> 
  void draw_axis (const QColor& color  = BLUE)
  {
    draw_edge<K> (typename K::Segment_2 ( typename  K::Point_2(0,0),
                                          typename  K::Point_2(0,1)),
                  color);
    draw_edge<K> (typename K::Segment_2 ( typename  K::Point_2(0,0),
                                          typename  K::Point_2(1,0)),
                  color);

    return;
  }
  template <typename K>
  void draw_path(std::vector<Reference_point<K> >& path,
                 typename Extended_polygon<K> & polygon,
                 bool to_draw_polygon)
  {
	double path_size = path.size();
	double color_step = (double)255/(double)path_size;
	double d_red=0;
	double d_green=255;

    Reference_point<K> ref_p;
    BOOST_FOREACH (ref_p,path)
    {
      int red   = d_red;
      int green = d_green;
      polygon.move_absolute(ref_p);
      if (to_draw_polygon)
      {        
        draw_polygon<K> (polygon.get_absolute_polygon(),QColor(red,green,0));
      }
      else //!(to_draw_polygon)
      {
        K::Point_2 polygon_ref_point = get_reference_point(polygon.get_absolute_polygon());        
        draw_cross<K>(polygon_ref_point ,QColor(red,green,0),0.025);
      }
      
      d_red+=color_step;
      d_green-=color_step;                
    }

    return;
  }
   template <typename K>
   void draw_path(std::vector<std::pair<Reference_point<K>, Reference_point<K> > >& path,
                 typename Extended_polygon<K> & polygon_a,
                 typename Extended_polygon<K> & polygon_b,
                 bool to_draw_polygons)
  {
    std::vector<Reference_point<K> > path_a, path_b;
    std::pair<Reference_point<K>, Reference_point<K> > ref_p;
    
    BOOST_FOREACH (ref_p, path)
    {
      path_a.push_back(ref_p.first);
      path_b.push_back(ref_p.second);  
    }

    draw_path(path_a, polygon_a, to_draw_polygons);
    draw_path(path_b, polygon_b, to_draw_polygons);
    return;
  }
  template <typename K>
  void save_path(std::vector<Reference_point<K> >& path,
                 typename Extended_polygon<K> & polygon,
                 const std::vector <CGAL::Polygon_2<K> >& workspace,
                 bool to_draw_polygon,
                 const std::string& path_name)
  {
	double path_size = path.size();
	double color_step = (double)255/(double)path_size;
	double d_red=0;
	double d_green=255;

    Reference_point<K> ref_p;
    BOOST_FOREACH (ref_p,path)
    {
      int red   = d_red;
      int green = d_green;
      polygon.move_absolute(ref_p);
      
      clear();
      draw_polygons<K>(workspace,BLUE);

      if (to_draw_polygon)
        draw_polygon<K> (polygon.get_absolute_polygon(),QColor(red,green,0));
      else //!(to_draw_polygon)
        draw_cross<K>(get_reference_point(polygon.get_absolute_polygon()) ,QColor(red,green,0),0.025);

      save_scene(path_name);
            
      d_red+=color_step;
      d_green-=color_step;                
    }

    return;
  }
  template <typename K>
  void save_path(std::vector<std::pair<Reference_point<K>, Reference_point<K> > >& path,
                 typename Extended_polygon<K> & polygon_a,
                 typename Extended_polygon<K> & polygon_b,
                 const std::vector <CGAL::Polygon_2<K> >& workspace,
                 bool to_draw_polygons,
                 const std::string& path_name)
  {
    std::pair<Reference_point<K>, Reference_point<K> > ref_p;
    BOOST_FOREACH (ref_p, path)
    {
      polygon_a.move_absolute(ref_p.first);
      polygon_b.move_absolute(ref_p.second);
      
      clear();
      draw_polygons<K>(workspace,BLUE);

      if (to_draw_polygons)
      {
        draw_polygon<K> (polygon_a.get_absolute_polygon(),RED);
        draw_polygon<K> (polygon_b.get_absolute_polygon(),GREEN);
      }
      else //!(to_draw_polygon)
      {
        draw_cross<K>(get_reference_point(polygon_a.get_absolute_polygon()), RED,0.025);
        draw_cross<K>(get_reference_point(polygon_b.get_absolute_polygon()), GREEN,0.025);
      }

      save_scene(path_name);       
    }
    return;
  }
  template <typename K>
  void draw_path_dbg(std::vector<Reference_point<K> >& path,
                     typename K::Point_2 & v)
  {
	double path_size = path.size();
	double color_step = (double)255/(double)path_size;
	double d_red=0;
	double d_green=255;

    Reference_point<K> ref_p;
    BOOST_FOREACH (ref_p,path)
    {
      int red   = d_red;
      int green = d_green;
      
      K::Point_2 rotated (rotate_point<K> (v,ref_p.get_rotation()));
      K::Point_2 moved (translate_point<K>(rotated,ref_p.get_location()));

      K::Segment_2 rod(ref_p.get_location(),moved);
      draw_edge<K> (rod,QColor(red,green,0));
      
      d_red+=color_step;
      d_green-=color_step;                
    }

    return;
  }
  
  template <typename K>
  void draw_path_dbg(std::vector<Reference_point<K> >& path,
                     typename K::Segment_2 & e)
  {
	double path_size = path.size();
	double color_step = (double)255/(double)path_size;
	double d_red=0;
	double d_green=255;

    Reference_point<K> ref_p;
    BOOST_FOREACH (ref_p,path)
    {
      int red   = d_red;
      int green = d_green;

      K::Point_2 rotated1,moved1,rotated2,moved2;
      rotated1 = rotate_point<K> (e.source(),ref_p.get_rotation());
      moved1 = translate_point<K>(rotated1,ref_p.get_location());
      rotated2 = rotate_point<K> (e.target(),ref_p.get_rotation());
      moved2 = translate_point<K>(rotated2,ref_p.get_location());

      K::Segment_2 edge(moved1,moved2);
      K::Segment_2 rod1(ref_p.get_location(),moved1);
      K::Segment_2 rod2(ref_p.get_location(),moved2);

      draw_edge<K> (edge,QColor(red,green,0));
      draw_edge<K> (rod1,QColor(red,green,0));
      draw_edge<K> (rod2,QColor(red,green,0));
      
      d_red+=color_step;
      d_green-=color_step;                
    }

    return;
  }
 private:
  template <typename NT> 
  double to_screen_double_x(const NT& x)
  {
    return ((CGAL::to_double (x) - _min_x) * _ratio_x);
  }
  template <typename NT> 
  double to_screen_double_y(const NT& y)
  {
    return (- (CGAL::to_double (y) - _min_y) * _ratio_y);
  }
  template <typename NT> 
  double to_file_double_x(const NT& x)
  {
    return (CGAL::to_double (x));
  }
  template <typename NT> 
  double to_file_double_y(const NT& y)
  {
    return (-CGAL::to_double (y));
  }

  

};

extern Graphics* global_graphics;

#endif  //Graphics_H
