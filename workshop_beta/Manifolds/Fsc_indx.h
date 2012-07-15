#ifndef FSC_INDX_ONE_ROBOT
#define FSC_INDX_ONE_ROBOT

#include "MMSTypedefs.h"

namespace mms{

//An fsc indx stores the nescesary data in order
//to retrieve the fsc
template <typename K>
class Fsc_indx
{
public:
  Fsc_indx ()
    : _type(NOT_VALID),_manifold_id(NO_ID),_fsc_id(NO_ID)
  {}
  Fsc_indx (Constraint_type type, int manifold_id, int fsc_id)
    : _type(type), _manifold_id(manifold_id), _fsc_id(fsc_id)
  {}
  Fsc_indx & operator= (const Fsc_indx & other)
  {
    if (this != &other) // protect against invalid self-assignment
    {
      _type         = other._type;
      _manifold_id  = other._manifold_id;
      _fsc_id       = other._fsc_id;
    }
    // by convention, always return *this
    return *this;
  }
  bool operator== (const Fsc_indx& other) const
  {
    if ((_type == NOT_VALID) && (other._type == NOT_VALID))
      return true;
    return ((_type        == other._type)     && 
            (_manifold_id == other._manifold_id)  && 
            (_fsc_id      == other._fsc_id));

  }
  bool operator!= (const Fsc_indx& other) const
  {
    if ((_type == NOT_VALID) && (other._type == NOT_VALID))
      return false;
    return ((_type        != other._type)     || 
            (_manifold_id != other._manifold_id)  || 
            (_fsc_id      != other._fsc_id) );

  }
  bool operator< (const Fsc_indx& other) const
  {
    if (_type != other._type)
      return (_type < other._type);
    
    if (_manifold_id != other._manifold_id)
      return (_manifold_id < other._manifold_id);

    return (_fsc_id < other._fsc_id);
  }

  bool is_valid()
  {
    return (_type != NOT_VALID);
  }

  void print()
  {
    std::cout <<"type: "<<_type
              <<" manifold_id: "<<_manifold_id
              <<" fsc_id: "<<_fsc_id
              <<std::endl;
  }
public:
  Constraint_type     _type;       
  int                 _manifold_id;
  int                 _fsc_id;  
}; //Fsc_indx

template <typename K>
struct Less_than_fsc_indx {
  bool operator()(const Fsc_indx<K>& fi1, const Fsc_indx<K>& fi2) const 
  {
    return (fi1<fi2);
  }
}; //Fsc_indx_one_robot
} //namespace mms{
#endif //FSC_INDX_ONE_ROBOT