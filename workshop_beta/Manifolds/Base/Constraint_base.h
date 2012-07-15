#ifndef CONSTRAINT_H
#define CONSTRAINT_H

#include "Manifolds\MMSTypedefs.h"
namespace mms{

template <typename Restriction,typename Region_of_interest>
class Constraint_base 
{
public:
  typedef Restriction         Restriction;
  typedef Region_of_interest  RoI;
  typedef Constraint_type     Type;
private:
  Type            _type;
  Restriction     _restriction;
  RoI             _roi;
  bool            _is_legal;
  bool            _is_roi_defined;
public:
  //Ctrs
  Constraint_base(Constraint_type type = NOT_VALID)
    :_is_legal(false) {}
  Constraint_base(const Restriction& restriction, Constraint_type type)
    :_restriction(restriction),_type(type),_is_roi_defined(false),_is_legal(true)
  {}
  Constraint_base(const Restriction& restriction, const RoI& roi,Constraint_type type)
    :_restriction(restriction),_roi(roi),_type(type),_is_roi_defined(true),_is_legal(true)
  {}

  //access functions
  const Restriction& restriction() const
  {
    CGAL_precondition(_is_legal);
    return (_restriction);
  }
  Type type() const
  {
    return (_type);
  }
  const Region_of_interest& region_of_interest() const
  {
    CGAL_precondition(_is_legal);
    return (_roi);
  }
  bool is_roi_defined() const 
  {
    CGAL_precondition(_is_legal);
    return (_is_roi_defined);
  }

  bool is_legal() const
  {
    return _is_legal;
  }
};  //class Constraint_base
} //namespace mms{
#endif //CONSTRAINT_H