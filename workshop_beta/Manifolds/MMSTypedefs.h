#ifndef MMS_TYPEDEFS_H
#define MMS_TYPEDEFS_H

namespace mms{

  const int NOT_FOUND = -1;
  const int NO_ID = -1;

  typedef std::pair<int,int> Int_pair;
  typedef 
  enum Constraint_type 
  {
    NOT_VALID,
    FIXED_ANGLE,
    FIXED_POINT
  };
} //namespace mms{

#endif  //MMS_TYPEDEFS_H