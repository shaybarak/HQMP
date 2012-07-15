#ifndef ROTATION_H
#define ROTATION_H

#include <CGAL/number_utils.h>

enum Angle_type 
{
  RAD,
  DEG
}; //Angle_type

template <typename NT>
class Rotation
{
public:
  //constructors
  Rotation () : _complete_rotation(true){}
  Rotation (const Rotation<NT> & r)
      : _sin (r.sin()),_cos (r.cos()),_complete_rotation(r.has_complete_rotation()) {}
  Rotation (const NT& s,const NT& c) : _sin(s), _cos (c),_complete_rotation (false){}

  //assignment operator
  Rotation<NT>& operator= (const Rotation<NT> &other)
  {
    if (this != &other) 
    {
      _sin = other.sin();
      _cos = other.cos();
      _complete_rotation = other.has_complete_rotation();
    }
    return *this;
  }

  //unary operators
  Rotation<NT> operator-() const
  {
    if (_complete_rotation ==true)
      return (Rotation<NT>());
    return Rotation<NT> (-_sin,_cos);
  }
  Rotation<NT> operator* (const Rotation<NT> &other) const
  {
    if (other.has_complete_rotation() == true)
      return *this;
    if (_complete_rotation == true)
      return other;

    NT s (_sin * other.cos() + _cos * other.sin());
    NT c (_cos * other.cos() - _sin * other.sin());

    return Rotation<NT> (s,c);
  }

  //comparison operators
  bool operator!= (const Rotation<NT> &other) const 
  {
    if ((other.has_complete_rotation() == true) && (_complete_rotation == true))
      return false;
    if ((CGAL::compare(_sin,other.sin()) != CGAL::EQUAL) ||
        (CGAL::compare(_cos,other.cos()) != CGAL::EQUAL) )
        return true;
    return false;
  }

  bool operator== (const Rotation<NT> &other) const 
  {
    if (other.has_complete_rotation() != _complete_rotation )
      return false;
    if ((other.has_complete_rotation() == true) && (_complete_rotation == true))
      return true;

    if ((CGAL::compare(_sin,other.sin()) == CGAL::EQUAL) &&
        (CGAL::compare(_cos,other.cos()) == CGAL::EQUAL) )
        return true;
    return false;
  }
  bool is_larger_than(const Rotation<NT> other) const 
  {
    //handle complete rotation
    CGAL_precondition (other.has_complete_rotation() != true);
    CGAL_precondition (_complete_rotation != true);

    //handle equality
    if (*this == other)
      return false;

    //handle zero
    if (this->is_zero_rotation())   //our angle is zero
      return false;
    if (other.is_zero_rotation())	//other angle is zero
      return true;

    //our angle is between 0 and 90;
    if ((CGAL::sign(_sin) != CGAL::NEGATIVE) && 
        (CGAL::sign(_cos) != CGAL::NEGATIVE))
    {
      if ((CGAL::sign(other.sin()) != CGAL::NEGATIVE) && 
          (CGAL::sign(other.cos()) != CGAL::NEGATIVE))
      {
        if (CGAL::compare(_cos,other.cos()) == CGAL::SMALLER)
          return true;
        else 
          return false;
      }
      else
      {
        return false;
      }
    }
    
    //our angle is between 90 and 180;
    else if ( (CGAL::sign(_sin) != CGAL::NEGATIVE) && 
              (CGAL::sign(_cos) != CGAL::POSITIVE) )
    {
      if ( (CGAL::sign(other.sin()) != CGAL::NEGATIVE) && 
           (CGAL::sign(other.cos()) != CGAL::NEGATIVE) )
           return true;
      else if ( (CGAL::sign(other.sin()) != CGAL::NEGATIVE) & 
                (CGAL::sign(other.cos()) != CGAL::POSITIVE) )
      {
        if (CGAL::compare(_cos,other.cos()) == CGAL::SMALLER)
          return true;
        else 
          return false;
      }
      else
        return false;
    }

    //our angle is between 180 and 270;
    else if ( (CGAL::sign(_sin) != CGAL::POSITIVE) &&
              (CGAL::sign(_cos) != CGAL::POSITIVE) )
    {
      if (CGAL::sign(other.sin()) != CGAL::NEGATIVE)
        return true;
      else if (CGAL::sign(other.cos()) != CGAL::POSITIVE)
      {
        if (CGAL::compare(_cos,other.cos()) == CGAL::LARGER)
          return true;
        else 
          return false;
      }
      else
        return false;
    }

    //our angle is between 270 and 360;
    else if ( (CGAL::sign(_sin) != CGAL::POSITIVE) &&
              (CGAL::sign(_cos) != CGAL::NEGATIVE))
    {
      if ( (CGAL::sign(other.sin()) != CGAL::POSITIVE) && 
           (CGAL::sign(other.cos()) != CGAL::NEGATIVE) )
      {
        if (CGAL::compare(_cos,other.cos()) == CGAL::LARGER)
          return true;
        else 
          return false;
      }
      else
        return true;
    }

    //not to be reached;
    CGAL_postcondition (false);
    return false;
  }
  //IO
  void print_exact() const
  {
    if (_complete_rotation)
	    cout <<"complete rotation" <<endl;
    else
	    cout <<"(" <<_sin<<" , "<<_cos<<")";
    return;
  }
  void print() const
  {
    if (_complete_rotation)
      cout <<"complete rotation" <<endl;
    else
      cout <<"(" <<CGAL::to_double(_sin)<<" , "<<CGAL::to_double(_cos)<<")";
    return;
  }
  void print_nice() const
  {
    if (_complete_rotation)
	    cout <<"complete rotation" ;
    else
	    cout << to_angle() ;
    return;
  }
  double to_angle(Angle_type mode = DEG) const
  {
    CGAL_precondition(_complete_rotation == false);

    double angle (atan2(CGAL::to_double(_sin),
                        CGAL::to_double(_cos)) 
                        * 180.0 / PI);
    if (angle < 0) 
      angle+=360;
    return (mode == DEG) ? angle : (angle * PI / 180);
  }
public:
  //access
  NT& sin() 
  {
    return _sin;
  }
  NT sin() const
  {
    return _sin;
  }
  NT& cos()
  {
    return _cos;
  }
  NT cos() const
  {
    return _cos;
  }
  bool has_complete_rotation () const
  {
    return _complete_rotation;
  }
  bool is_zero_rotation() const
  {
    // zero rotation is (0,1)
    if (_complete_rotation)
      return false;
    if (CGAL::sign(_sin) != CGAL::ZERO) 
      return false;
    if (CGAL::compare(_cos,1) != CGAL::EQUAL)
      return false;
    return true;
  }
  bool is_pi() const
  {
    // pi rotation is (0,-1)
    if (_complete_rotation)
      return false;
    if (CGAL::sign(_sin) != CGAL::ZERO) 
      return false;
    if (CGAL::compare(_cos,-1) != CGAL::EQUAL)
      return false;
    return true;
  }
private:
  NT _sin;
  NT _cos;
  bool _complete_rotation; //indicates that there is no rotation constraint
};

template <typename NT>
struct Less_than_rotation
{
  bool operator()(const Rotation<NT>& r1,const Rotation<NT>& r2) const 
  {
    return (( r1.is_larger_than(r2) || (r1 == r2)) ?
            false:
            true);
  }
}; //Less_than_rotation

template <typename NT>
struct Less_than_rotation_pair
{
  typedef Rotation<NT>                  Rotation;
  typedef std::pair<Rotation, Rotation> Rotation_pair;
  bool operator()(const Rotation_pair& p1, 
                  const Rotation_pair& p2) const 
  {
    if (p1.first != (p2.first))
      return (p2.first.is_larger_than(p1.first));

    return (( p1.second.is_larger_than(p2.second) || (p1.second == p2.second)) ?
            false:
            true);      
  }
}; //Less_than_rotation_pair


#endif  //ROTATION_H
