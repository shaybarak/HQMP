#pragma once

/*#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2/Gps_default_dcel.h>
#include <CGAL/Polygon_set_2.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Arr_non_caching_segment_basic_traits_2.h>*/

#include <vector>
#include <map>
#include <list>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/foreach.hpp>

#include <QVector>
#include <QPointF>

/*#include <CGAL/General_polygon_set_2.h>
#include <CGAL/Gps_segment_traits_2.h>
#include <CGAL/Boolean_set_operations_2/Gps_default_dcel.h>*/

//#define foreach         BOOST_FOREACH

//#pragma warning ( disable : 4819 )

using namespace std;

/*******************************************************************************************
 * This file contatins basic typedefs (from CGAL and more).
 *******************************************************************************************/

//typedef CGAL::Gmpq                                  Number_type;
//typedef CGAL::Exact_predicates_inexact_constructions_kernel				Kernel;
//typedef Kernel::Point_2													Point_2;
//typedef	std::vector<Point_2>											Container;
//typedef CGAL::Arr_non_caching_segment_basic_traits_2<Kernel>			Traits_2;
////typedef CGAL::Gps_default_dcel<Traits_2>								Dcel_;
//
//typedef CGAL::Polygon_set_2<Kernel>					Polygon_set_2;
//typedef Polygon_set_2::Arrangement_2                Arrangement_2;
//typedef Polygon_set_2::Polygon_2					Polygon_2;
//typedef Polygon_set_2::Polygon_with_holes_2			Polygon_with_holes_2;
//
//typedef Point_2										Conf;
//typedef vector<Conf>								ConfSet;
//typedef list<Polygon_2>								Obstacles;
//typedef pair<Point_2,Point_2>						Room;
//typedef pair<int,int>								Edge;
//typedef list<Edge>									Edges;
//typedef pair<ConfSet,Edges>							ConfGraph;
//
//typedef	pair<Edge,int>								Move;
//typedef	list<Edge>									MoveRecorder;
//
//typedef list<vector<Conf>>							Path;
//
//typedef QVector<QVector<QPointF>>					QTPath;
//typedef	pair<vector<pair<Conf,Conf>>,ConfSet>		GeometricGraph;

typedef pair<QPointF,double>	QConf;