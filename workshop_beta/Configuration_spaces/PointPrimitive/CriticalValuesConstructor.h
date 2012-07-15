#ifndef CRITICAL_VALUES_CONSTRUCTOR_H
#define CRITICAL_VALUES_CONSTRUCTOR_H

#include "Utils\ReferencePoint.h"
#include "Utils\Rotation_utils\Rotation.h"
#include "Utils\Rotation_utils\RotationRange.h"

#include "Utils\Interval_utils\Interval.h"
#include "Utils\Interval_utils\IntervalSet.h"

namespace FixedPoint
{
template <typename K, typename AK, typename AK_conversions>
class Critical_values_constructor 
{
public:
  typedef typename K::FT                        NT;
  typedef typename K::Point_2                   Point;
  typedef typename K::Segment_2                 Segment;
  typedef Rotation<typename NT>                 Rotation;
  typedef typename Rotation_range_absolute<NT>  Rotation_range;

  typedef CGAL::Sqrt_extension<typename NT, 
                               typename NT>        Root_of_2;

  typedef unsigned int                                  Multiplicity;
  typedef typename AK::Coefficient                      Coefficient;
  typedef typename AK::Algebraic_real_1                 Algebraic_real_1;
  typedef typename AK::Polynomial_1                     Polynomial;
  typedef CGAL::Polynomial_traits_d<Polynomial>         PT;
  typedef CGAL::Polynomial<typename NT>                 Poly_rat_1;
  
  typedef typename AK::Solve_1                          Solve_1;
  typedef typename AK::Construct_algebraic_real_1       Construct_algebraic_real_1;
  
private:
  enum Contact_type
  {
    ROBOT_V__OBSTACLE_E,
    ROBOT_E__OBSTACLE_V,
    UNKNOWN
  };
  class Value_inducing_data
  {
  public:
    Value_inducing_data() 
      : type (UNKNOWN) {}
    Contact_type type;            //either from robot vertex and obstacle edge or robot edge & obstacle vertex
    Point        v;               //vertex of robot     or  obstacle
    Segment      e;               //edge   of obstacle  or  robot
  };//Value_inducing_data
private:  
  Point                 _center;           //robots center onto which the vertices are relative to
  Point                 _fixed_ref_point;  //robot's fixed reference pointcenter;
  const Rotation_range& _rotation_range;

  AK&                         _ak;
  Construct_algebraic_real_1  _construct_algebraic_real_1;
  Solve_1                     _solve;
  AK_conversions              _ak_convertor;

  Value_inducing_data _data;     
public:
  Critical_values_constructor(const Point& c, const Point& p, 
                              const Rotation_range& rotation_range,
                              AK &ak)
    : _center(c), _fixed_ref_point(p) ,_rotation_range(rotation_range), 
      _ak(ak), _ak_convertor(ak)
  {
    _construct_algebraic_real_1 = _ak.construct_algebraic_real_1_object();
    _solve = _ak.solve_1_object();
  }

  //vertex - edge
  template <typename OutputIterator>
  void construct_values(const Point& v, const Segment& e,OutputIterator& oi)
  {
    _data.type = ROBOT_V__OBSTACLE_E;
    _data.v = v;
    _data.e = e;

    return construct_values (oi);
  }

  //edge - vertex 
  template <typename OutputIterator>
  void construct_values(const Segment& e,const Point& v,OutputIterator& oi)
  {
    _data.type = ROBOT_E__OBSTACLE_V;
    _data.v = v;
    _data.e = e;

    return construct_values (oi);
  }

private:
  template <typename OutputIterator>
  void construct_values(OutputIterator& oi)
  {
    if (!candidate_contact())
      return;

    Poly_rat_1 poly;
    get_coeficients(poly);

    std::list<Root_of_2> unfiltered_values, filtered_values;
    solve (poly, std::back_inserter(unfiltered_values)); //get contact values
    
    for (std::list<Root_of_2>::iterator iter = unfiltered_values.begin();
         iter != unfiltered_values.end();
         ++iter)
    {
      if (is_valid_value(*iter))
        filtered_values.push_back(*iter);
    }

    
    if (filtered_values.empty())
      return;

    //convert Root_of_2 to Algebraic_real_1
    CGAL_postcondition ( filtered_values.size() <= 2);
    if (filtered_values.size() == 2)
      contruct_algebraic_roots( poly, oi);
    else //filtered_values.size() == 1
      contruct_algebraic( filtered_values.front(), oi);

    return;
  }
  bool candidate_contact()
  {
    CGAL_precondition (_data.type != UNKNOWN);
    if (_data.type == ROBOT_V__OBSTACLE_E)
      return candidate_contact_ve();
    else
      return candidate_contact_ev();
  }
  bool candidate_contact_ve()
  {
    CGAL_precondition (_data.type == ROBOT_V__OBSTACLE_E);

    //in order for the case to be relevant
    //the obstacle edge should be closer to the fixed reference point than
    //the distance between the vetex and the relative reference point (which is zero)
    if ( CGAL::compare (CGAL::squared_distance (_data.e,_fixed_ref_point) , 
                        CGAL::squared_distance (_data.v,_center))
                        == CGAL::LARGER )
      return false;
    else
      return true;
  }

  bool candidate_contact_ev()
  {
    CGAL_precondition (_data.type == ROBOT_E__OBSTACLE_V);
  
    //in order for the case to be relevant
    //the obstacle vertex should be closer to the fixed reference point 
    //than the largest distance between the robot edge and the relative reference point
    //this is the distance of the furthest robot endpoint

    NT d_s_sq = CGAL::squared_distance (_data.e.source(), _center);
    NT d_t_sq = CGAL::squared_distance (_data.e.target() ,_center);
            
    NT d_sq   = CGAL::squared_distance (_data.v ,_fixed_ref_point);
    
    if ( ( CGAL::compare (d_sq,d_s_sq) == CGAL::LARGER  ) &&
         ( CGAL::compare (d_sq,d_t_sq) == CGAL::LARGER  ))
         return false;
    else
      return true;
  }

  void get_coeficients(Poly_rat_1& poly)
  {        
    CGAL_precondition (_data.type != UNKNOWN);
    if (_data.type == ROBOT_V__OBSTACLE_E)
      get_coeficients_ve(poly);
    else
      get_coeficients_ev(poly);

    return;
  }
  void get_coeficients_ve(Poly_rat_1& poly)
  {
    CGAL_precondition (_data.type == ROBOT_V__OBSTACLE_E);
    
    //////////////////////////////////////////////
    //                                          //
    //   k_2 T^2 + k_1 T  + k_0 = 0             //
    //                                          //
    // k_2 =  (x_q - x_o1 - x_i) delta_oy -     //
    //        (y_q - y_o1 - y_i) delta_ox       //
    // k_1 = -2(y_i delta_oy + x_i delta_ox)    //
    // k_0 =  (x_q - x_o1 + x_i) delta_oy -     //
    //        (y_q - y_o1 + y_i) delta_ox       //
    //                                          //
    // delta_ox = x_o2 - x_o1                   //
    // delta_oy = y_o2 - y_o1                   //
    //////////////////////////////////////////////

    const Point& p_q  (_fixed_ref_point);
    const Point& p_o1 (_data.e.source());
    const Point& p_o2 (_data.e.target());
    const Point& p_i  (_data.v);
    
    NT dx =  p_o2.x() - p_o1.x();
    NT dy =  p_o2.y() - p_o1.y();
        
    NT k_2 = (p_q.x() - p_o1.x() - p_i.x()) * dy -
             (p_q.y() - p_o1.y() - p_i.y()) * dx;
    NT k_1 = -2*(p_i.y()*dy + p_i.x()*dx);
    NT k_0 = (p_q.x() - p_o1.x() + p_i.x()) * dy -
             (p_q.y() - p_o1.y() + p_i.y()) * dx;

    Poly_rat_1 x (CGAL::shift(Poly_rat_1(1),1));
    poly = k_2*x*x + k_1*x + k_0;
    return;
  }


  void get_coeficients_ev(Poly_rat_1& poly)
  {
    CGAL_precondition (_data.type == ROBOT_E__OBSTACLE_V);
    
    //////////////////////////////////////////////
    //                                          //
    //  l_2 T^2 + l_1 T  + l_0 = 0              //
    //                                          //
    // l_2 =    a delta_y - b delta_x + k       //
    // l_1 = -2(a delta_x + b delta_y)          //
    // l_0 = -l_2 + 2k                          //
    //                                          //
    // delta_x = x_2 - x_1                      //
    // delta_y = y_2 - y_1                      //
    // a = (x_o - x_q)                          //
    // b = (y_o - y_q)                          //
    // k = x_1*y_2 - x_2*y_1                    //
    //////////////////////////////////////////////

    const Point& p_q  (_fixed_ref_point);
    const Point& p_1 (_data.e.source());
    const Point& p_2 (_data.e.target());
    const Point& p_o  (_data.v);
    
    NT dx =  p_2.x() - p_1.x();
    NT dy =  p_2.y() - p_1.y();
    NT a =  p_o.x() - p_q.x();
    NT b =  p_o.y() - p_q.y();
    NT k =  p_1.x()*p_2.y() - p_2.x()*p_1.y();
        
    NT l_2 =     a * dy - b * dx + k;
    NT l_1 = -2*(a * dx + b * dy);
    NT l_0 = -l_2 +2*k;

    Poly_rat_1 x (CGAL::shift(Poly_rat_1(1),1));
    poly = l_2*x*x + l_1*x + l_0;
    return;
  }
  bool is_valid_value(const Root_of_2& t)
  {
    if (is_in_roi(t) == false)
      return false;
    CGAL_precondition (_data.type != UNKNOWN);
    if (_data.type == ROBOT_V__OBSTACLE_E)
      return is_valid_value_ve(t);
    else
      return is_valid_value_ev(t);
  }
  bool is_in_roi(const Root_of_2& t)
  {
    if (_rotation_range.is_full_range())
      return true;
    NT start = get_parametrization_theta<K>(_rotation_range.get_start_rotation());
    NT end = get_parametrization_theta<K>(_rotation_range.get_end_rotation());

    Root_of_2 s(start, NT(0), t.root());
    Root_of_2 e(end,   NT(0), t.root());

    CGAL_precondition (start != end);
    if (start < end)
    {
      if ((s <= t) && (t<=e))
        return true;
      else
        return false;
    }
    else // (end < start)
    {
      if ((t >= s) || (t<=e))
        return true;
      else
        return false;
    }
  }
  bool is_valid_value_ve(const Root_of_2& t)
  {
    //////////////////////////////////////////////////
    //                                              //
    //  s should be in [0,1]                        //
    //                                              //
    //      (1-t^2) x_i - 2t y_i + (1+t^2)(x_q-x_o1)//
    // s   =  --------------------------------------//
    //              (1+t^2)(x_o2 - x_01)            //
    //////////////////////////////////////////////////

    const NT& x_q  (_fixed_ref_point.x());
    const NT& x_o1 (_data.e.source().x());
    const NT& x_o2 (_data.e.target().x());
    const NT& x_i  (_data.v.x());
    const NT& y_i  (_data.v.y());

    Root_of_2 t_sq = t*t;
    Root_of_2 zero(0, 0, t_sq.root());
    Root_of_2 one(1, 0, t_sq.root());
    
    if (CGAL::compare (x_o2, x_o1) != CGAL::EQUAL)
    {
      Root_of_2 s_numer =((1-t_sq)*x_i - 2*t*y_i + (1+t_sq)*(x_q-x_o1));// / ((1+t_sq)(x_o2 - x_o1));
      Root_of_2 s_denom =((1+t_sq)*(x_o2 - x_o1));
      Root_of_2 s = s_numer / s_denom;

      if (  (CGAL::compare(s, zero) == CGAL::SMALLER) ||
            (CGAL::compare(s, one) == CGAL::LARGER) )
            return false;

      return true;
    }
    else //(CGAL::compare (x_o2, x_o1) != CGAL::EQUAL)
    {
      //////////////////////////////////////////////////
      //                                              //
      //  s should be in [0,1]                        //
      //                                              //
      //      2t x_i + (1-t^2) y_i + (1+t^2)(y_q-y_o1)//
      // s   =  --------------------------------------//
      //              (1+t^2)(y_o2 - y_01)            //
      //////////////////////////////////////////////////

      const NT& y_q  (_fixed_ref_point.y());
      const NT& y_o1 (_data.e.source().y());
      const NT& y_o2 (_data.e.target().y());
      
      Root_of_2 s_numer =((2*t)*x_i - (1-t_sq)*y_i + (1+t_sq)*(y_q-y_o1));
      Root_of_2 s_denom =((1+t_sq)*(y_o2 - y_o1));
      Root_of_2 s = s_numer / s_denom;

      if (  (CGAL::compare(s, zero) == CGAL::SMALLER) ||
            (CGAL::compare(s, one) == CGAL::LARGER) )
            return false;

      return true;
    }
  }

  bool is_valid_value_ev(const Root_of_2& t)
  {
    //////////////////////////////////////////////////
    //                                              //
    //  s should be in [0,1]                        //
    //                                              //
    //      x_o - x_1(p,t)                          //
    // s   =  --------------------                  //
    //      x_2(p,t) - x_1(p,t)                     //
    //                                              //
    // x(p,t) = [(1-t^2) / (1 + t^2)] x -           //
    //          [ 2t     / (1 + t^2)] y +           //
    //                                x_q           //
    //////////////////////////////////////////////////
    
    Root_of_2 t_sq (t*t);
    Root_of_2 denom  (1+t_sq);
    Root_of_2 a_numer(1-t_sq);
    Root_of_2 a = a_numer / denom;
    Root_of_2 b_numer (2*t);
    Root_of_2 b = b_numer / denom;

    NT x_q  (_fixed_ref_point.x());
    NT x_o  (_data.v.x());
    Root_of_2  x_1  ( a*_data.e.source().x() - 
                      b*_data.e.source().y() +
                      x_q);
    Root_of_2  x_2  ( a*_data.e.target().x() - 
                      b*_data.e.target().y() +
                      x_q);    
    
    if (CGAL::compare(x_1, x_2) != CGAL::EQUAL)
    {
      Root_of_2 s    ((x_o - x_1) / (x_2 - x_1));

      if (  (CGAL::compare(s,0) == CGAL::SMALLER) ||
            (CGAL::compare(s,1) == CGAL::LARGER) )
            return false;

      return true;
    }
    else //(CGAL:::compare(x1, x_2) != CGAL::EQUAL)
    {
      //////////////////////////////////////////////////
      //                                              //
      //  s should be in [0,1]                        //
      //                                              //
      //      y_o - y_1(p,t)                          //
      // s   =  --------------------                  //
      //      y_2(p,t) - y_1(p,t)                     //
      //                                              //
      // x(p,t) = [ 2t      / (1 + t^2)] x +          //
      //          [ (1-t^2) / (1 + t^2)] y +          //
      //                                 x_q          //
      //////////////////////////////////////////////////

      NT y_q  (_fixed_ref_point.y());
      NT y_o  (_data.v.y());
      Root_of_2  y_1  ( b*_data.e.source().x() + 
                        a*_data.e.source().y() +
                        y_q);
      Root_of_2  y_2  ( b*_data.e.target().x() + 
                        a*_data.e.target().y() +
                        y_q);    
      CGAL_postcondition (CGAL::compare(y_2, y_1) != CGAL::EQUAL);
      Root_of_2 s    ((y_o - y_1) / (y_2 - y_1));

      if (  (CGAL::compare(s,0) == CGAL::SMALLER) ||
            (CGAL::compare(s,1) == CGAL::LARGER) )
            return false;

      return true;
    }
  }
private:
  template <typename OutputIterator>
  void contruct_algebraic(const Root_of_2& t, OutputIterator& oi)
  {
    //for a number t = a_0 x + a_1 sqrt (r)
    //it is one of the roots of the equation
    //x^2 - 2a_0 x + a_0^2 - a_1^2 r
    //
    //if a_1 > 0 it is the larger root
    //if a_1 < 0 it is the smaller root
    CGAL_precondition ( !CGAL::is_zero(t.a1()) && 
                        !CGAL::is_zero(t.root()) );

    
    NT c_2 (1);
    NT c_1 (-2*t.a0());
    NT c_0 (t.a0()*t.a0() - t.a1()*t.a1()*t.root());

    Poly_rat_1 x (CGAL::shift(Poly_rat_1(1),1));
    Poly_rat_1 rat_poly = c_2*x*x + c_1*x + c_0;

    //    convert and solve
    Polynomial poly;
    _ak_convertor.convert_polynomial(rat_poly, poly);

    if (CGAL::sign(t.a1()) == CGAL::POSITIVE)
      *oi = _construct_algebraic_real_1(poly, 1);  //take higher root
    else //(CGAL_sign(t.a1()) == CGAL::NEGATIVE)
      *oi = _construct_algebraic_real_1(poly, 0);  //take lower root
    ++oi;
  }
  template <typename OutputIterator>
  void contruct_algebraic_roots(const Poly_rat_1 &rat_poly, OutputIterator& oi)

  {
    //    convert and solve
    Polynomial poly;
    _ak_convertor.convert_polynomial(rat_poly, poly);
    solve(poly, oi);
    return;
  }

  template <typename OutputIterator>
  void solve (const Poly_rat_1 poly, OutputIterator& oi)
  {
    CGAL_precondition (poly.degree() <= 2);
    if (poly.degree() == 2)
    {
      //poly = ax^2+bx+c
      NT alpha =-poly[1] / (2*poly[2]); //   -b/2a
      NT betta = CGAL::abs(1 / (2*poly[2])); //   +-1/2a
      NT gamma = poly[1]*poly[1] - 4* poly[2]*poly[0];  //    b^2-4ac
      if (CGAL::is_zero(gamma)) // one solution
      {
        *oi = Root_of_2(alpha); 
        ++oi;
      }
      else if (CGAL::sign(gamma) == CGAL::POSITIVE) //two solutions
      {
        *oi = Root_of_2(alpha, -betta, gamma);
        ++oi;
        *oi = Root_of_2(alpha,  betta, gamma);
        ++oi;
      }
    }
    else if (poly.degree() == 1)
    {
      //poly = ax+b
      *oi = Root_of_2(-poly[0] / (2*poly[1])); // -b/2a
      ++oi;
    }
    return;
  }
  template <typename OutputIterator>
  void solve (const Polynomial poly,OutputIterator& oi)
  {
    typedef typename AK::Multiplicity_type              Multiplicity;
    typedef std::pair<Algebraic_real_1, Multiplicity>   Res;
    std::vector <Res > res;
    _solve (poly, std::back_inserter(res));

    for (unsigned int i(0); i < res.size(); ++i)
    {
      *oi = res[i].first;
      oi++;
    }
  }
}; //Critical_values_constructor 
} //namespace FixedPoint


#endif //CRITICAL_VALUES_CONSTRUCTOR_H