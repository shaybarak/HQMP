#ifndef ROTATION_UTILS_T_H
#define ROTATION_UTILS_T_H

#include "Utils\Rotation_utils\Rotation.h"


enum ApproximationType	{CRUDE,FINE};

template <typename NT> 
Rotation<NT> approximate_rotation(const NT& sin, const NT& cos,
                                  ApproximationType type = CRUDE)
{
  //if sin & cos are approximation of sine and cosine 
  //then this will return a consistent approximation
  
  NT dirx (cos) ;
  NT diry (sin) ;

  NT sin_num, cos_num, denom;
  if (type == CRUDE)
  {
    CGAL::rational_rotation_approximation<NT>(dirx, diry, 
                                              sin_num, cos_num, denom, 
                                              EPS_NUM, EPS_DEN_CRUDE);
  }
  else //type == FINE
  {
    CGAL::rational_rotation_approximation<NT>(dirx, diry, 
                                              sin_num, cos_num, denom, 
                                              EPS_NUM, EPS_DEN_FINE);
  }
  return Rotation<NT> (sin_num/denom, cos_num/denom );
}

template <typename NT> 
Rotation<NT> approximate_rotation(const NT& sin, const NT& cos, 
                                  bool smaller, ApproximationType type = CRUDE)
{
  typedef Rotation<NT> Rotation;
  static Rotation small_fine_rotation (SMALL_SINE_FINE,  SMALL_COSINE_FINE);
  static Rotation small_crude_rotation(SMALL_SINE_CRUDE, SMALL_COSINE_CRUDE);

  Rotation rot (approximate_rotation(sin, cos, type));
  Rotation small_rotation( (type == CRUDE)? small_crude_rotation : small_fine_rotation );
  
  if (rot.is_zero_rotation())
    return rot;

  if (smaller == true)
  {
    //first make sure that small_rotation is smaller than rot
    if (rot.is_larger_than(small_rotation) == false)
      return rot;
    
    small_rotation = -small_rotation;
    //return an approximate angle SMALLER than (sin,cos)
    while (rot.is_larger_than(Rotation(sin,cos))==true)	//not exact as (sin,cos) are not exact!
      rot = rot * small_rotation;
    
  } //(smaller == false)
  else
  {
    //return an approximate angle LARGER than (sin,cos)
    while (rot.is_larger_than(Rotation(sin,cos))==false)	//not exact as (sin,cos) are not exact!
      rot = rot * small_rotation;
    
  }
  return rot;
}
template <typename NT> 
Rotation<NT> to_rotation(double angle, Angle_type mode = RAD ,ApproximationType type = CRUDE)
{
  //convert to radians if needed
  if (mode == DEG)
      angle = angle * PI/180;

  //get approximate values for the sine and cosine
  NT sin (sin(angle));
  NT cos (cos(angle));
  bool smaller ( (angle >= 0) ? true: false);
  return approximate_rotation<NT>(sin, cos, smaller, type);
}
#endif //ROTATION_UTILS_T_H
