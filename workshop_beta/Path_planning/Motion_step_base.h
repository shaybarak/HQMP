#ifndef MOTION_STEP_BASE_H
#define MOTION_STEP_BASE_H

#include "Utils\ReferencePoint.h"

template <typename K>
class Motion_step_base
{
public:
  enum Motion_type {UNINITIALIZED, TRANSLATION, ROTATION, STATIC}; 
  typedef typename Reference_point<K>   Reference_point;
protected:
  Motion_type     motion_type;
public:
  Motion_step_base (Motion_type type) : motion_type(type) 
  {}
  Motion_type     type() const {return motion_type;}
public:
  virtual Reference_point source() const = 0;
  virtual Reference_point target() const = 0;

  virtual void read(std::istream& is)  = 0;
  virtual void write(std::ostream& os) = 0;
  
  virtual void reverse_motion_step() =0;
  virtual double approximate_motion_time(double speed) =0;
protected:
  virtual bool is_legal_motion() = 0;
}; //Motion_step_base
#endif //MOTION_STEP_BASE_H