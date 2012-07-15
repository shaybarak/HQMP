#include "stdafx.h"
#include "ConnectedComponents.h"

/////////////////
//CTR's & DTR's//
/////////////////

Connected_components::Connected_components()
:_id(0)
{
  elem_cc_map.clear();
  id_cc_map.clear();
  return;
}

Connected_components::~Connected_components()
{}

/////////////////
//set functions//
/////////////////

void Connected_components::add_element(int elem_id)
{
  CGAL_precondition (elem_cc_map.find(elem_id) == elem_cc_map.end());
  
  int cc_id = get_new_cc_id();
  Connected_component cc;
  cc.insert(elem_id);
  
  elem_cc_map[elem_id] = cc_id;
  id_cc_map[cc_id] = cc;
  return; 
}
void Connected_components::connect_elements(int elem_id1,int elem_id2)
{
  CGAL_precondition (elem_cc_map.find(elem_id1) != elem_cc_map.end());
  CGAL_precondition (elem_cc_map.find(elem_id2) != elem_cc_map.end());

  int cc1_id;
  Int_int_map_iter  iter1 = elem_cc_map.find(elem_id1);
  cc1_id = iter1->second;

  int cc2_id;
  Int_int_map_iter  iter2 = elem_cc_map.find(elem_id2);
  cc2_id = iter2->second;

  //if in same cc return;
  if (cc1_id == cc2_id)
    return ;
  
  Connected_component& cc2 = id_cc_map[cc2_id];

  //merge into int_set_ptr1
  id_cc_map[cc1_id].insert(cc2.begin(),cc2.end());
  BOOST_FOREACH (int id , cc2 )
    elem_cc_map[id] = cc1_id;

  id_cc_map.erase(cc2_id);

  return;
}

/////////////////
//get functions//
/////////////////
std::set<int>& Connected_components::get_connected_component(int elem_id)
{
  CGAL_precondition (elem_cc_map.find(elem_id)!=elem_cc_map.end());
  return id_cc_map[elem_cc_map[elem_id]];
}
int            Connected_components::get_connected_component_id(int elem_id)
{
  return ( (elem_cc_map.find(elem_id)!=elem_cc_map.end()) ?
            elem_cc_map[elem_id] :
            -1);
}
std::map<int,std::set<int> >&  Connected_components::get_connected_components()
{
  return id_cc_map;
}
bool Connected_components::is_in_same_connected_component(int elem_id1,int elem_id2)
{
  Int_int_map_iter iter1 = elem_cc_map.find(elem_id1);
  Int_int_map_iter iter2 = elem_cc_map.find(elem_id2);
  if (  (iter1 == elem_cc_map.end()) ||
        (iter2 == elem_cc_map.end())  )
        return false;
  return (iter1->second == iter2->second);
}
/////////////////////
//dbg functions//
/////////////////////
void Connected_components::print_connected_components()
{
  std::cout   <<"There are "<< id_cc_map.size() <<" connected components"<<std::endl;
  std::cout   <<"cc id"<<"|"<<"cc elemnts"<<std::endl;
  std::cout   <<"-----"<<"|"<<"--------------------"<<std::endl;
  for (Int_cc_map::iterator iter = id_cc_map.begin(); iter != id_cc_map.end(); ++iter)
  {
    print_connected_component(iter->first);
  }
}
void Connected_components::print_connected_component(int cc_id)
{
  Int_cc_map::iterator iter = id_cc_map.find(cc_id);
  if (iter == id_cc_map.end())
    return;
  
  std::cout <<iter->first<<"    |";
  BOOST_FOREACH (int elem,iter->second)
    std::cout<<elem<<", ";
  std::cout<<std::endl;
}

/////////////////////
//private functions//
/////////////////////
int Connected_components::get_new_cc_id()
{
  return _id++;
}