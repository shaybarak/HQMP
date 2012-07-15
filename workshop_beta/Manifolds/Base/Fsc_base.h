#ifndef FSC_BASE_H
#define FSC_BASE_H

#include "Manifolds\MMSTypedefs.h"

namespace mms{

template <typename Cell>
class Fsc_base 
{
public:
  typedef Cell            Cell;
private:
  Cell       _cell;
public:
  //constructors
  Fsc_base() {}
  Fsc_base( const Cell& cell)
    :_cell(cell)
  {}
  Fsc_base(const Fsc_base& other) 
    :_cell(other.cell())
  {}
  const Cell& cell() const
  {
    return _cell;
  }
  Cell& cell() 
  {
    return _cell;
  }
};  //Fsc_base


} //namespace mms{

#endif //FSC_BASE_H