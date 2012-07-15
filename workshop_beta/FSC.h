#ifndef FREE_SPACE_CELL_ONE_ROBOT
#define FREE_SPACE_CELL_ONE_ROBOT

#include "Utils\ReferencePoint.h"

#include "Manifolds\MMSTypedefs.h"
#include "Manifolds\Fixed_angle\Fixed_angle_constraint.h"
#include "Manifolds\Fixed_angle\Fixed_angle_fsc.h" 
#include "Manifolds\Fixed_point\Fixed_point_constraint.h"
#include "Manifolds\Fixed_point\Fixed_point_fsc.h"  

#include "Configuration_spaces\PointPrimitive\Fixed_point_utils.h"
namespace mms{

template <typename _K, typename _AK, typename _AK_conversions>
class FSC
{
public:
  typedef _K                                                K;
  typedef _AK                                               AK;
  typedef _AK_conversions                                   AK_conversions;
  typedef Reference_point<K>                                Ref_p;
public: //for debugging
  typedef Fixed_point_fsc<K, AK, AK_conversions>            Fsc_fp;
  typedef Fixed_point_constraint<K>                         Constraint_fp;
  typedef Fixed_angle_fsc<K>                                Fsc_fa;
  typedef Fixed_angle_constraint<K>                         Constraint_fa;
private:
  Constraint_type _type;
  CGAL::Object    _feature_obj;
  CGAL::Object    _constraint_obj;  
public:
  //constructors
  FSC() {};
  FSC(const Fsc_fp& fsc, const Constraint_fp& constraint)
    :_type(FIXED_POINT), 
     _feature_obj(CGAL::make_object(fsc)),
     _constraint_obj(CGAL::make_object(constraint))
  {}
  FSC(const Fsc_fa& fsc, const Constraint_fa& constraint)
    :_type(FIXED_ANGLE), 
     _feature_obj(CGAL::make_object(fsc)),
     _constraint_obj(CGAL::make_object(constraint))
  {}
  FSC(const FSC& other) 
    :_type(other.get_constraint_type()),
    _feature_obj(other.get_feature_object()),
    _constraint_obj(other.get_constraint_obj()),
  {}
public:
  Constraint_type  get_constraint_type() const
  {
    return _type;
  }
  CGAL::Object  get_feature_object() const
  {
    return _feature_obj;
  }
  CGAL::Object  get_constraint_obj() const
  {
    return _constraint_obj;
  }
public:
  template <typename T>  T get_free_space_feature() const
  {
    CGAL_precondition(_feature_obj.empty() == false);
    CGAL_precondition(_feature_obj.is<T>());
    return CGAL::object_cast<T>(_feature_obj);
  }
  template <typename T>  T get_free_space_constraint() const
  {
    CGAL_precondition(_constraint_obj.empty() == false);
    CGAL_precondition(_constraint_obj.is<T>());

    return CGAL::object_cast<T>(_constraint_obj);
  }

public: //dbg
  bool contains(Ref_p ref_p) const
  { 
    if (_type == NOT_VALID)
      return false;
    

    if (_type == FIXED_ANGLE)
    {
      Fsc_fa        fsc         = get_free_space_feature<Fsc_fa>();
      Constraint_fa constraint  = get_free_space_constraint<Constraint_fa>();

      if (ref_p.get_rotation() != constraint.restriction())
        return false;

      if (fsc.cell().is_in_polygon(ref_p.get_location(), true) == false)
        return false;

      return true;
    }

    if (_type == FIXED_POINT)
    {
      Fsc_fp        fsc         = get_free_space_feature<Fsc_fp>();
      Constraint_fp constraint  = get_free_space_constraint<Constraint_fp>();

      if (ref_p.get_location() != constraint.restriction())
        return false;
      
      AK_conversions ak_conversions;
      //AK ak;
      //typename AK::Construct_algebraic_real_1 construct_algebraic_real_1 = ak.construct_algebraic_real_1_object();

      typename K::FT tau = FixedPoint::get_parametrization_theta<K> (ref_p.get_rotation());
      if (fsc.cell().contains(ak_conversions.convert(tau)) == false)
        return false;

      return true;
    }
    CGAL_precondition(false);
    return false;
  }
};  //FSC


} //namespace mms{
#endif //FREE_SPACE_CELL_ONE_ROBOT