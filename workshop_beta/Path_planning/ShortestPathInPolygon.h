#ifndef SHORTEST_PATH_IN_POLYGON_H
#define SHORTEST_PATH_IN_POLYGON_H
 
#include "Graph\Graph.h"
#include "Utils\Number_utils\Sqrt_approximation.h"
#include "Utils\logging.h"
#include <CGAL/Constrained_Delaunay_triangulation_2.h> 
 
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
  typedef std::vector<typename Polygon>           Polygon_vec;
  typedef typename Polygon_vec::const_iterator Iter;
 
  typedef Graph<typename K::Point_2,Less_than_points<K> >        Vertex_Graph;
  typedef std::vector<typename K::Point_2>  Point_vec;
  typedef typename Point_vec::iterator      Point_vec_iter;
  typedef std::list<typename K::Point_2>    Point_list;
  typedef typename K::Segment_2       Segment_2;
  typedef typename Polygon::Vertex_circulator      V_circulator;
  typedef typename Polygon::Vertex_const_iterator  V_iterator;
  typedef typename Polygon_with_holes::Hole_const_iterator H_iterator;
  typedef CGAL::Constrained_Delaunay_triangulation_2<K> Delaunay;
  typedef typename Delaunay::Face_iterator Face_iterator;
  typedef typename Delaunay::Face Face;
  typedef typename Delaunay::Face_handle Face_handle;
  typedef typename Delaunay::Vertex Vertex;
  typedef typename Vertex::Vertex_handle Vertex_handle;
  typedef typename Delaunay::Edge_iterator Edge_iterator;
  typedef typename Delaunay::Finite_edges_iterator Edge_iterator_finite;
  typedef typename Delaunay::Finite_vertices_iterator Vertex_iterator_finite;
  typedef typename Delaunay::Edge Edge;
 
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
  void shortest_path(const Point & source,const Point & target, Point_list& path, const Polygon_with_holes& polygon)
  {	
	TIMED_TRACE_ENTER("shortest_path (Path_planning folder)");
		
	Polygon polygon_outer_boundary(polygon.outer_boundary());

	if (polygon.has_holes()){
		Segment_2 straigh_line_path(source, target);
		//If there is a straight-line solution, return with it immediately.
		if (is_in_polygon(straigh_line_path, polygon,true)){
			path.push_front(target);
			path.push_front(source);
			return;
		}else{
				
			Delaunay tr;
			if (!_initialized){

				//triangulate polygon
				for (H_iterator hi = polygon.holes_begin  (); hi!= polygon.holes_end (); ++hi){
					insert_consrtaints(Polygon(*hi), tr);
				}
				insert_consrtaints(polygon_outer_boundary, tr);
				this->construct_dual_graph(tr,source,target);
				_initialized = true;
			}
			Point_list point_path;
			_graph.find_path(source,target,point_path);
			refine_path(point_path, path, tr);
			TIMED_TRACE_EXIT("path_in_polygon (Path_planning folder)");
			return;
		}
	}

	path_in_polygon(source,target,path, polygon_outer_boundary);
	return;
	

  }
 

  private:

   void insert_consrtaints(Polygon &polygon, Delaunay& tr){
		V_circulator start =  polygon.vertices_circulator();
		V_circulator cur = start;
		V_circulator next = cur;
		next++;
		while(next != start){
			tr.insert_constraint(*cur, *next);
			cur++;
			next++;
		}
		tr.insert_constraint(*cur, *next);
 }

  //find a path in a simple polygon with no holes
  void path_in_polygon(const Point & source,const Point & target, Point_list& path, const Polygon& polygon)
  {
	TIMED_TRACE_ENTER("path_in_polygon (Path_planning folder)");
    if (polygon.is_convex()){
		path.push_front(target);
		path.push_front(source);
		return;
	}else{
		//If there is a straight-line solution, return with it immediately.
		Segment_2 straigh_line_path(source, target);
		if (is_in_polygon(straigh_line_path, polygon,true)){
			path.push_front(target);
			path.push_front(source);
			return;
		}
	}
	
	follow_polygon_edges(source,target,polygon,path);
	TIMED_TRACE_EXIT("path_in_polygon (Path_planning folder)");
	return;
		      
  }



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
    double weight (CGAL::to_double(approx_d));
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
        TIMED_TRACE_ENTER("init_graph");
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
        TIMED_TRACE_EXIT("init_graph");
  }
 
void follow_polygon_edges(const Point &source,const Point & target,  const Polygon& polygon,Point_list& point_path){
       
	     
	TIMED_TRACE_ENTER("follow_polygon_edges"); 
    Point_list path1 ;
    Point_list path2 ;
    double path1_weight  = 0 , path2_weight  = 0 ;
 
    K::FT sq_dist ;
    K::FT approx_d;
 
    V_circulator v1;
    V_circulator v2;  
               
    get_closest_reachable_vertex(source,polygon, v1);
    get_closest_reachable_vertex(target,polygon,v2) ;  
               
    path1.push_front(source);
    path2.push_front(target);
       
    V_circulator cur = v1;
               
    //go from v1 to v2 cw
    while(cur != v2){
		path1.push_back(*cur) ;
		sq_dist = CGAL::squared_distance(*cur,*(cur+1));
		approx_d = _sqrt_approximation(sq_dist) ;
		path1_weight += CGAL::to_double(approx_d) ;
		cur++;
    }
    //go from v1 to v2 ccw
    while(cur != v1){
		path2.push_front(*cur) ;
		sq_dist = CGAL::squared_distance(*cur,*(cur+1));
		approx_d = _sqrt_approximation(sq_dist) ;
		path2_weight += CGAL::to_double(approx_d) ;
		cur++;
    }
 
    path1.push_back(*v2);
    path1.push_back(target);
         
    path2.push_front(*v1);
    path2.push_front(source);
               
    //decide on path direction: cw or ccw
    if (path1_weight < path2_weight){
        point_path = path1 ;
    }else {
        point_path = path2 ;
    }
	TIMED_TRACE_EXIT("follow_polygon_edges");
 
 }
 
void construct_dual_graph(Delaunay& tr,const Point & source,const Point & target){
 
       
    TIMED_TRACE_ENTER("build_dual_graph (Path_planning folder)");
    Edge_iterator_finite edge_it = tr.finite_edges_begin(),
                    edge_beyond = tr.finite_edges_end();
    
	while(edge_it != edge_beyond){
		
		Edge edge = *edge_it;
		Face_handle face = edge.first;
		int nbIndex = edge.second;
		Face_handle neighbor = face->neighbor(nbIndex);
		if (tr.is_infinite(face) || tr.is_infinite(neighbor)){
				edge_it++;
				continue;
		}
		if (!(is_face_inside_polygon(face)) || !(is_face_inside_polygon(neighbor))){
			edge_it++;
			continue;
		}
               
		Point center_face1(centroid(face->vertex(0)->point(),face->vertex(1)->point(),face->vertex(2)->point()));
		if (_graph.is_in_graph(center_face1) == false)
		{
			this->insert_vertex(center_face1);
			_vertices.push_back(center_face1);
		}
           
		Point center_face2(centroid(neighbor->vertex(0)->point(),neighbor->vertex(1)->point(),neighbor->vertex(2)->point()));
		if (_graph.is_in_graph(center_face2) == false)
		{
			this->insert_vertex(center_face2);
			_vertices.push_back(center_face2);
		}
		
		this->insert_edge(center_face1, center_face2);  
		edge_it++;
    }
 
    //connect source to graph
    TIMED_TRACE_ACTION("build_dual_graph", "start check source");
    if (_graph.is_in_graph(source) == false){
		Face_handle source_face = tr.locate(source);
			this->insert_vertex(source);
		_vertices.push_back(source);
		Point source_face_center(centroid(source_face->vertex(0)->point(),source_face->vertex(1)->point(),source_face->vertex(2)->point()));
		if (_graph.is_in_graph(source_face_center) == false)
		{
			this->insert_vertex(source_face_center);
			_vertices.push_back(source_face_center);
		}
		this->insert_edge(source, source_face_center);
    }
    TIMED_TRACE_ACTION("build_dual_graph", "finish check source");
 
	 //connect target to graph
    TIMED_TRACE_ACTION("build_dual_graph", "start check target");  
    if (_graph.is_in_graph(target) == false){
		this->insert_vertex(target);
		_vertices.push_back(target);
		Face_handle target_face = tr.locate(target);
		Point target_face_center(centroid(target_face->vertex(0)->point(),target_face->vertex(1)->point(),target_face->vertex(2)->point()));
		if (_graph.is_in_graph(target_face_center) == false)
		{
			this->insert_vertex(target_face_center);
			_vertices.push_back(target_face_center);
		}
		this->insert_edge(target_face_center, target);
    }
    TIMED_TRACE_ACTION("build_dual_graph", "finish check target");
	
	TIMED_TRACE_EXIT("build_dual_graph");

}
 
//connect vertices at midpoints of triangulation edges
void refine_path(Point_list& old_path, Point_list& refined_path,Delaunay& tr){
       
    Point_list path;
    Point_list::iterator it_cur;
    Point_list::iterator it_next;
        
	it_cur = old_path.begin();
    it_next = old_path.begin();
    path.push_front(*it_cur);
	Face_handle cur_face = tr.locate(*it_cur);
	Face_handle next_face;
 
    it_next++;
    while(it_next !=  old_path.end()){
		next_face = tr.locate(*it_next, cur_face);
		if (next_face == cur_face){
			it_next++;
			continue;
		}
		Point p;
		mid_point_of_common_edge(cur_face,next_face,p);
		if (p != old_path.front()){
			path.push_back(p);
		}
		cur_face = next_face;
		it_next++;          
    }
	if (old_path.back() != path.back()){
		path.push_back(old_path.back());
	}
    refined_path = path;
}
 
//find the middle point of the common edge of two adjacent facets
void mid_point_of_common_edge(Face_handle face1, Face_handle face2, Point& mid_point){
        
	Point v1;
    Point v2;
       
	int i;
	//find the first common vertex
	for(i = 0 ; i < 3; i++){
		if ((*face2).has_vertex(face1->vertex(i))){
			v1 = face1->vertex(i)->point();
			break;
		}
	}
    //find the second common vertex 
	for(int j = i+1; j < 3; j++){
		if ((*face2).has_vertex(face1->vertex(j))){
			v2 = face1->vertex(j)->point();
			break;
		}
	}
 
	mid_point = midpoint(v1,v2);

}
 

void print_triangle(Face_handle f){
        for(int i = 0 ; i < 3; i++){
                 std::cout <<f->vertex(i)->point().x() << " , " << f->vertex(i)->point().y();
                cout<<"   ";
        }
        cout<<endl;
}
 
void print_point(Point& p){
         std::cout << p.x() << " , " << p.y() << std::endl;
}


bool is_face_inside_polygon(Face_handle face){
	
	for (int i = 0 ; i<3; i++){
		if(!is_in_polygon(face->vertex(i)->point(), _polygon, true)){
			return false;
		}
	}
	Segment_2 edge1(face->vertex(0)->point(),face->vertex(1)->point());
	Segment_2 edge2(face->vertex(1)->point(),face->vertex(2)->point());
	Segment_2 edge3(face->vertex(2)->point(),face->vertex(0)->point());
	if (!is_in_polygon(edge1, _polygon, true)|| !is_in_polygon(edge2, _polygon, true) || !is_in_polygon(edge3, _polygon, true)){
		return false;
	}

	return true;

}

 
 
}; //Shortest_path_in_polygon
 
#endif // SHORTEST_PATH_IN_POLYGON_H