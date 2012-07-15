#ifndef SHORTEST_PATH_IN_POLYGON_H
#define SHORTEST_PATH_IN_POLYGON_H

#include "Graph\Graph.h"
#include "Utils\Number_utils\Sqrt_approximation.h"


template <typename K>
class Shortest_path_in_polygon
{
private:
  template <typename K>
  struct Less_than_points
  {
    bool operator()(const typename K::Point_2 p1, 
                    const typename K::Point_2 p2) const 
    {
      return (( CGAL::compare_xy(p1,p2) == CGAL::SMALLER ) ?
              true:
              false);
    }
  };
private:
  typedef typename K::Point_2               Point;
  typedef CGAL::Polygon_2<K>                Polygon;
  typedef CGAL::Polygon_with_holes_2<K>     Polygon_with_holes;

  typedef Graph<typename K::Point_2,Less_than_points<K> >        Vertex_Graph;
  typedef std::vector<typename K::Point_2>  Point_vec;
  typedef typename Point_vec::iterator      Point_vec_iter;
  typedef std::list<typename K::Point_2>    Point_list;
private:
  bool                  _initialized;
  Polygon_with_holes    _polygon;
  Vertex_Graph          _graph;
  Point_vec             _vertices;

  Sqrt_approximation<typename K::FT>    _sqrt_approximation;
public:
  //ctrs
  Shortest_path_in_polygon(const Polygon& polygon)
    :_initialized(false),_polygon(polygon)
  {
      //lazy construction: do nothing
  }
  Shortest_path_in_polygon(const Polygon_with_holes& polygon)
    :_initialized(false),_polygon(polygon)
  {
    CGAL_precondition (polygon.is_unbounded() == false);
    //lazy construction: do nothing
  }

  //get shortest path
  void shortest_path(const Point & source,const Point & target, Point_list& path)
  {
    if (!_initialized)
    {
      this->init_graph();
      _initialized=true;
    }
    //check if source is in graph
    if (_graph.is_in_graph(source) == false)
    {
      //add vertex
      this->insert_vertex(source);
      _vertices.push_back(source);
      //insert relevant edges to graph
      BOOST_FOREACH(Point v, _vertices)
      {
        if (source !=v)
          if (is_in_polygon<K>(K::Segment_2(source,v),_polygon,true))
            this->insert_edge(source,v);
      }
    }

    //check if target is in graph
    if (_graph.is_in_graph(target) == false)
    {
      //add vertex
      this->insert_vertex(target);
      _vertices.push_back(target);
      //insert relevant edges to graph
      BOOST_FOREACH(Point v, _vertices)
      {
        if (target !=v)
          if (is_in_polygon<K>(K::Segment_2(target,v),_polygon,true))
            this->insert_edge(target,v);
      }
    }

  _graph.find_path(source,target, path);
  return;

  }
private:
  void add_polygon_vertices(const Polygon& pgn)
  {
    for (Polygon::Vertex_iterator iter (pgn.vertices_begin()); iter != pgn.vertices_end(); ++iter)
    {
      _vertices.push_back(*iter);
    }
    return;
  }
  void get_polygon_vertices()
  {
    CGAL_precondition(_vertices.size() == 0);
    add_polygon_vertices(_polygon.outer_boundary());

    for (Polygon_with_holes::Hole_const_iterator iter = _polygon.holes_begin() ; iter != _polygon.holes_end(); ++iter)
      add_polygon_vertices(*iter);

    return;
  }
  void insert_polygon_edges(const Polygon& pgn)
  {
    for (Polygon::Edge_const_iterator iter (pgn.edges_begin()); iter != pgn.edges_end(); ++iter)
    {
      insert_edge(iter->source(), iter->target());
    }
    return;
  }
  void insert_polygon_edges()
  {
    insert_polygon_edges(_polygon.outer_boundary());

    for (Polygon_with_holes::Hole_const_iterator iter = _polygon.holes_begin() ; iter != _polygon.holes_end(); ++iter)
      insert_polygon_edges(*iter);

    return;
  }
  void insert_edge(const Point& v1, const Point& v2)
  {
    K::FT sq_dist (CGAL::squared_distance(v1,v2));
    K::FT approx_d(_sqrt_approximation(sq_dist));
    int weight (CGAL::to_double(approx_d));
    _graph.add_edge(v1,v2,weight);
    return;
  }

  void insert_vertex(const Point& v)
  {
    _graph.add_vertex(v);
    return;
  }
  void init_graph()
  {
    this->get_polygon_vertices();
    
    //insert vertices
    BOOST_FOREACH (Point v, _vertices)
      insert_vertex(v);
    
    //iterate over all vertices couples and insert appropriate ones
    for (Point_vec_iter iter1 = _vertices.begin(); iter1 != _vertices.end(); ++iter1)
    {
      for (Point_vec_iter iter2 = iter1; iter2 != _vertices.end(); ++iter2)
      {
        if (*iter1 != *iter2)
          if (is_in_polygon<K>(K::Segment_2(*iter1,*iter2),_polygon,true))
          {
            this->insert_edge(*iter1,*iter2);
            //global_graphics->draw_edge<K> (*iter1,*iter2,RED);
          }
      }
    }
  }
}; //Shortest_path_in_polygon

#endif // SHORTEST_PATH_IN_POLYGON_H