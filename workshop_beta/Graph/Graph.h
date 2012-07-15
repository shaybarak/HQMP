#ifndef GRAPH_H
#define GRAPH_H
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>

#include <map>
#include <boost/config.hpp>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/connected_components.hpp>

#include "ConnectedComponents.h"

#include "Utils\Random_utils\RandomUtils.h"

template <typename Node, typename Is_Less_than = std::less<Node> >
class Graph
{
private:
  typedef boost::property<boost::edge_weight_t, double>                  Weight_property;  
  typedef boost::adjacency_list<boost::vecS,          //container type for the out edge list
                                boost::vecS,          //container type for the vertex list
                                boost::undirectedS,   //directed / undirected
                                int,                  //vertex properties
                                Weight_property       //edge properties
                                >  Boost_graph;		//slightly more time efficient
  typedef boost::graph_traits < Boost_graph >::vertex_descriptor      Vertex_descriptor;
  typedef boost::graph_traits<Boost_graph>::edge_iterator             Edge_iterator;
private:
	Boost_graph*                    _graph;
    std::vector<Vertex_descriptor>  _parent;
    std::vector<double>             _distance;
    std::vector<int>                _component; 

    std::map<int, Node>             _id_node_map;
    std::map<Node, int>             _node_id_map;
    int                             _curr_id;
private:
    bool                  _use_connected_components;
    Connected_components  _connected_components;
private:	
	/* this method will run Dijkstra on the current graph representation from the source input argument. 
	   the method can fail if the source is not the in the graph, return value indicates success or failure.
	   upon success this method updates the parent and distance data members.*/
    void run_dijkstra(const int source_id)
    {
      int   num_of_vertices (boost::num_vertices(*_graph));
      std::vector<Vertex_descriptor>    parent(num_of_vertices);      //will hold for each vertex its parent in the shortest path to source.
      std::vector<double>               distance(num_of_vertices);    //will hold for each vertex its distance in the shortest path to source.
      Vertex_descriptor                 source_descriptor(source_id);
    	
      //stores result at parent and distance.
      dijkstra_shortest_paths(*_graph, source_descriptor, boost::predecessor_map(&parent[0]).distance_map(&distance[0]));
      this->_parent=parent;
      this->_distance=distance;

      return ; 
    }

public:
  Graph(int initial_size = 0, int use_connected_components = false)
    :_curr_id(0), _use_connected_components(use_connected_components)
  {
    _graph = new Boost_graph(initial_size);
  }

  ~Graph(void)
  {
    delete _graph;
  }
	
  /* add vertex to the graph. the vertex property is its ID. */
  bool is_in_graph(const Node& node)
  {
    return (  (_node_id_map.find(node) != _node_id_map.end()) ?
              true :
              false);
  }
  void add_vertex(const Node& node)
  {
    if (is_in_graph(node))
      return; //node is in the graph

    int id = get_unique_id();
    _node_id_map[node]  = id;
    _id_node_map[id]    = node;

    add_vertex_to_graph (id);
    if (_use_connected_components)
      _connected_components.add_element(id);

    return;
  }
  void add_edge(const Node& node1 , const Node& node2 , const double weight=1)
  {
    CGAL_precondition (is_in_graph(node1)); //node1 is in the graph
    CGAL_precondition (is_in_graph(node2)); //node2 is in the graph
    
    int id1 (_node_id_map[node1]);
    int id2 (_node_id_map[node2]);

    add_edge_to_graph (id1,id2,weight);
    if (_use_connected_components)
      _connected_components.connect_elements(id1,id2);

    return;
  }
  
  /* this method runs Dijkstra algorithm on the graph.	   */
  void find_path(const Node& source, const Node& target, std::list<Node>& path)
  {
    CGAL_precondition (is_in_graph(source)); //source is in the graph
    CGAL_precondition (is_in_graph(target)); //target is in the graph

    int source_id (_node_id_map[source]);
    int target_id (_node_id_map[target]);

    if (_use_connected_components &&
        !_is_in_same_cc(source_id,target_id))
        return;

    run_dijkstra(source_id);
    int curr_node_id =target_id;

    //if target configuration parent is itself, there is no path from source to target.
    if(_parent[curr_node_id]==curr_node_id) 
      return;

    CGAL_precondition (_id_node_map.find(curr_node_id) != _id_node_map.end());
    path.push_front(_id_node_map[curr_node_id]);

    while(_parent[curr_node_id] != curr_node_id )
    {
      curr_node_id = _parent[curr_node_id];
      CGAL_precondition (_id_node_map.find(curr_node_id) != _id_node_map.end());
      path.push_front (_id_node_map[curr_node_id]);
    }

    return ;
  }
public:
  //cc utils
  bool is_in_same_cc(const Node& source, const Node& target)
  {
    CGAL_precondition (_use_connected_components);
    CGAL_precondition (is_in_graph(source)); //source is in the graph
    CGAL_precondition (is_in_graph(target)); //target is in the graph

    int source_id (_node_id_map[source]);
    int target_id (_node_id_map[target]);

    return _is_in_same_cc(source_id,target_id);
  }
  int get_cc_id (const Node& node)
  {
    CGAL_precondition (_use_connected_components);
    CGAL_precondition (is_in_graph(node)); //source is in the graph
    return _connected_components.get_connected_component_id(_node_id_map[node]);
  }
  int num_of_connected_components() const
  {
    CGAL_precondition(_use_connected_components);
    return _connected_components.num_of_connected_components();
  }
  std::vector<std::vector<Node> > get_connected_components()
  {
    CGAL_precondition (_use_connected_components);
    Connected_components::Int_cc_map& ccs_ids = _connected_components.get_connected_components();
    Connected_components::Int_cc_map::iterator iter;

    std::vector<std::vector<Node> > ccs_nodes;
    for (iter = ccs_ids.begin(); iter!= ccs_ids.end(); ++iter)
    {
      std::vector<Node> cc;
      BOOST_FOREACH(int i,iter->second)
        cc.push_back(_id_node_map[i]);
      ccs_nodes.push_back(cc);
    }
    return ccs_nodes;
  }
  template<typename OutputIterator>
  void get_cc_nodes(int cc_id, OutputIterator& oi)
  {
    CGAL_precondition (_use_connected_components);
    Connected_components::Int_cc_map& ccs_ids = _connected_components.get_connected_components();
    Connected_components::Int_cc_map::iterator iter = ccs_ids.find(cc_id);

    CGAL_precondition(iter != ccs_ids.end());

    BOOST_FOREACH(int id, iter->second)
      *oi++ = _id_node_map[id];
    return ;
  }
  void print_connected_components()
  {
    _connected_components.print_connected_components();
    return;
  }
  //debug
  void print() const 
  {
     // get the property map for vertex indices
    typedef boost::property_map<Boost_graph, boost::vertex_index_t>::type Index_map;
    Index_map index = get(boost::vertex_index, *_graph);

    //print vertices
    std::cout << "graph vertices :" << std::endl;
    typedef boost::graph_traits<Boost_graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vp;
    for (vp = vertices(*_graph); vp.first != vp.second; ++vp.first)
      std::cout << index[*vp.first] <<  " ";
    std::cout << std::endl;

    //print edges
    std::cout << "graph edges : ";
    boost::graph_traits<Boost_graph>::edge_iterator ei, ei_end;
    for (tie(ei, ei_end) = edges(*_graph); ei != ei_end; ++ei)
        std::cout << "(" << index[source(*ei, *_graph)] 
                  << "," << index[target(*ei, *_graph)] << ") ";
    std::cout << std::endl;
    return;
  }
  template<typename Randomizer>
  Node get_random_node(Randomizer& rand)
  {
    CGAL_precondition(_id_node_map.empty() == false);
    int node_id = rand.get_random_int(0, _id_node_map.size() - 1);
    return (_id_node_map[node_id]);
  }

  template<typename Randomizer>
  Node get_random_node_in_same_cc(Randomizer& rand, Node& node)
  {
    CGAL_precondition (is_in_graph(node)); //source is in the graph
    int node_id = _node_id_map[node];

    int picked_id = rand.get_random_int(0, _id_node_map.size() - 1);
    int num_of_iterations=0;
    while (_is_in_same_cc(picked_id, node_id) == false)
    {
      if (num_of_iterations++ >20)
        return Node();
      picked_id = rand.get_random_int(0, _id_node_map.size() - 1);
    }
    return (_id_node_map[picked_id]);
  }
  template<typename Randomizer>
  Node get_random_node_in_maximal_cc(Randomizer& rand)
  {
    CGAL_precondition(_use_connected_components);
    int cc_id = _connected_components.get_maximal_cc_id();
    int node_id = rand.get_random_int(0, _id_node_map.size() - 1);
    while (_connected_components.get_connected_component_id(node_id) != cc_id)
      node_id = rand.get_random_int(0, _id_node_map.size() - 1);
    return (_id_node_map[node_id]);    
  }
private:
  int get_unique_id()
  {
    return _curr_id++;
  }
  void add_vertex_to_graph(int node_id)
  {
    boost::add_vertex(node_id,*_graph);
  }
  void add_edge_to_graph(const int id1,const int id2, const double weight=1)
  {
    Weight_property edge_property(weight);
    boost::add_edge(id1, id2, edge_property, *_graph);	
    return;
  }

  bool _is_in_same_cc(const int id1,const int id2)
  {
    CGAL_precondition(_use_connected_components);
    return (_connected_components.is_in_same_connected_component(id1,id2));
  }
};  //Graph

#endif  //GRAPH_H
