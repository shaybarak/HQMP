#ifndef CONNECTED_COMPONENTS_H
#define CONNECTED_COMPONENTS_H

#include <set>
#include <map>

class Connected_components
{
public:
  typedef std::set<int>                     Connected_component;
  typedef std::map<int,int>                 Int_int_map;
  typedef Int_int_map::iterator             Int_int_map_iter;
  typedef std::map<int,Connected_component> Int_cc_map;
  typedef Int_cc_map::iterator              Int_cc_map_iter;
public:
  //CTR & DTR
  Connected_components();
  ~Connected_components();
  //set functions
  void add_element(int elem_id);                    //add element to ds, the element is a connected component
  void connect_elements(int elem_id1,int elem_id2); //conect two connected components represented by two representing elements
  //get functions
  Connected_component& get_connected_component(int elem_id); //return all elements in element's connected component
  int                  get_connected_component_id(int elem_id);
  Int_cc_map&          get_connected_components();
  bool is_in_same_connected_component(int elem_id1,int elem_id2);
  int  num_of_connected_components() const
  {
    return id_cc_map.size();
  }
  //dbg
  void print_connected_components();
  void print_connected_component(int cc_id);
  int get_maximal_cc_id()
  {
    unsigned int max_id =0;
    unsigned int max_size=0;
    CGAL_precondition(id_cc_map.empty() == false);
    for (Int_cc_map_iter iter = id_cc_map.begin(); iter != id_cc_map.end(); ++iter)
    {
      if (iter->second.size() > max_size)
      {
        max_size = iter->second.size();
        max_id = iter->first;
      }
    }    
    return max_id;
  }
  
private:
  int get_new_cc_id();
private:
  Int_int_map   elem_cc_map;
  Int_cc_map    id_cc_map;
  int           _id;
}; // Connected_components
#endif //CONNECTED_COMPONENTS_H
