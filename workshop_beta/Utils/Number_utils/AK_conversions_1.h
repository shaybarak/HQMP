#ifndef AK_CONVERSIONS_1_H
#define AK_CONVERSIONS_1_H

#include <CGAL/Polynomial_traits_d.h>
#include <CGAL/Lazy_exact_nt.h>
#include <CGAL/Arithmetic_kernel.h>
#include <CGAL/Algebraic_kernel_d_1.h>

//---------------------------------------------------
//conversions between Algebraic_real_1 and Rational 
//---------------------------------------------------
template<typename Arithmetic_kernel = CGAL::CORE_arithmetic_kernel>
class Algebraic_kernel_d_1_conversions_rational
{
public:
  typedef typename Arithmetic_kernel::Integer			Integer;
  typedef typename Arithmetic_kernel::Rational          Rational;
  typedef typename CGAL::Algebraic_kernel_d_1<Integer>	AK;
  typedef typename AK::Algebraic_real_1	                Algebraic;
public:
  typedef typename Algebraic                            Exact_nt;
  typedef typename Rational                             Approximated_nt;
public:
  typedef CGAL::Polynomial<Approximated_nt>               Approximated_nt_polynomial;;
  typedef typename AK::Polynomial_1                       Exact_nt_polynomial;
private:
  AK*   ak_ptr;
  bool  to_delete;
public:
  //CTR's, DTR's
  Algebraic_kernel_d_1_conversions_rational()
    : to_delete(true)
  {
    ak_ptr = new AK();
  }
  Algebraic_kernel_d_1_conversions_rational(AK& ak)
    : ak_ptr(&ak), to_delete(false)
  {}
  ~Algebraic_kernel_d_1_conversions_rational()
  {
    if (to_delete)
      delete ak_ptr; 
  }
public: //conversion functions
  Rational to_rational(const Algebraic& a, bool larger = true, int  num_of_bits = 10)
  {
    return (larger ? (ak_ptr->approximate_relative_1_object()(a,num_of_bits).second) :
                     (ak_ptr->approximate_relative_1_object()(a,num_of_bits).first));
  }
  Rational rational_in_interval(const Algebraic& low, const Algebraic& high)
  {
    return (ak_ptr->bound_between_1_object()(low,high));
  }
  template<typename NT>
  Algebraic construct_algebraic(const NT& r)
  {
    typename AK::Construct_algebraic_real_1 construct_algebraic_real_1 = ak_ptr->construct_algebraic_real_1_object();
    return construct_algebraic_real_1(r);
  }
public: //same conversion functions that follow the concept required by Interval's point_in_interval
  Approximated_nt approximate(const Exact_nt& a, bool larger = true, int  num_of_bits = 10)
  {
    return to_rational(a, larger, num_of_bits);
  }
  Approximated_nt approximate_in_interval(const Exact_nt& low, const Exact_nt& high)
  {
    return rational_in_interval(low,high);
  }
  Exact_nt convert(const Approximated_nt& r)
  {
    return construct_algebraic(r);
  }
  void convert_polynomial(const Approximated_nt_polynomial& approximated_nt_polynomial, 
                          Exact_nt_polynomial& exact_nt_polynomial)
  {
    typedef CGAL::Fraction_traits <Approximated_nt_polynomial>      FT;
    typename AK::Coefficient denom;
    typename FT::Decompose()(approximated_nt_polynomial, exact_nt_polynomial, denom);
  }

}; //Algebraic_kernel_d_1_conversions_rational


template<typename Arithmetic_kernel = CGAL::CORE_arithmetic_kernel>
class Algebraic_kernel_d_1_conversions_lazy_nt_rational
{
public:
  typedef typename Arithmetic_kernel::Integer			  Integer;
  typedef typename Arithmetic_kernel::Rational            Rational;
  typedef typename CGAL::Lazy_exact_nt<typename Rational> NT;
  typedef typename CGAL::Algebraic_kernel_d_1<Integer>	  AK;
  typedef typename AK::Algebraic_real_1	                  Algebraic;
public:
  typedef typename Algebraic                              Exact_nt;
  typedef typename NT                                     Approximated_nt;
public:
  typedef CGAL::Polynomial<Approximated_nt>               Approximated_nt_polynomial;;
  typedef typename AK::Polynomial_1                       Exact_nt_polynomial;  
private:
  AK*   ak_ptr;
  bool  to_delete;
public:
  //CTR's, DTR's
  Algebraic_kernel_d_1_conversions_lazy_nt_rational()
    : to_delete(true)
  {
    ak_ptr = new AK();
  }
  Algebraic_kernel_d_1_conversions_lazy_nt_rational(AK& ak)
    : ak_ptr(&ak), to_delete(false)
  {}
  ~Algebraic_kernel_d_1_conversions_lazy_nt_rational()
  {
    if (to_delete)
      delete ak_ptr; 
  }
private: //conversion functions
  Rational to_rational(const Algebraic& a, bool larger = true, int  num_of_bits = 10)
  {
    return (larger ? (ak_ptr->approximate_relative_1_object()(a,num_of_bits).second) :
                     (ak_ptr->approximate_relative_1_object()(a,num_of_bits).first));
  }
  Rational rational_in_interval(const Algebraic& low, const Algebraic& high)
  {
    return (ak_ptr->bound_between_1_object()(low,high));
  }
  template<typename NT>
  Algebraic construct_algebraic(const NT& r)
  {
    typename AK::Construct_algebraic_real_1 construct_algebraic_real_1 = ak_ptr->construct_algebraic_real_1_object();
    return construct_algebraic_real_1(r);
  }
public: //same conversion functions that follow the concept required by Interval's point_in_interval
  Approximated_nt approximate(const Exact_nt& a, bool larger = true, int  num_of_bits = 10)
  {
    return Approximated_nt(to_rational(a, larger, num_of_bits));
  }
  Approximated_nt approximate_in_interval(const Exact_nt& low, const Exact_nt& high)
  {
    return Approximated_nt(rational_in_interval(low,high));
  }
  Exact_nt convert(const Approximated_nt& r)
  {
    return construct_algebraic(r.exact());
  }
  void convert_polynomial(const Approximated_nt_polynomial& approximated_nt_polynomial, 
                          Exact_nt_polynomial& exact_nt_polynomial)
  {
    typedef CGAL::Polynomial_traits_d<Approximated_nt_polynomial>   PT;
    typedef CGAL::Polynomial<Rational>                              Rat_polynomial;
    typedef CGAL::Fraction_traits <Rat_polynomial>                  FT;

    std::vector<Rational> rat_coefficients;
    for ( PT::Coefficient_const_iterator iter = approximated_nt_polynomial.begin();
          iter != approximated_nt_polynomial.end();
          ++iter)
    {
      rat_coefficients.push_back(iter->exact());
    }
    
    Rat_polynomial rat_polynomial(rat_coefficients.begin(), rat_coefficients.end());
    
    typename AK::Coefficient denom;
    typename FT::Decompose()(rat_polynomial, exact_nt_polynomial, denom);

    return;
  }

}; //Algebraic_kernel_d_1_conversions_lazy_nt_rational


template<typename Arithmetic_kernel = CGAL::CORE_arithmetic_kernel>
class Algebraic_kernel_d_1_conversions_double
{
public:
  typedef typename Arithmetic_kernel::Integer			Integer;
  typedef typename CGAL::Algebraic_kernel_d_1<Integer>	AK;
  typedef typename AK::Algebraic_real_1	                Algebraic;
public:
  typedef typename Algebraic                            Exact_nt;
  typedef typename double                               Approximated_nt;
public:
  typedef CGAL::Polynomial<Approximated_nt>             Approximated_nt_polynomial;;
  typedef typename AK::Polynomial_1                     Exact_nt_polynomial;  
private:
  AK*   ak_ptr;
  bool  to_delete;
public:
  //CTR's, DTR's
  Algebraic_kernel_d_1_conversions_double()
    : to_delete(true)
  {
    ak_ptr = new AK();
  }
  Algebraic_kernel_d_1_conversions_double(AK& ak)
    : ak_ptr(&ak), to_delete(false)
  {}
  ~Algebraic_kernel_d_1_conversions_double()
  {
    if (to_delete)
      delete ak_ptr; 
  }
public: //conversion functions
  template<typename NT>
  Algebraic construct_algebraic(const NT& r)
  {
    typename AK::Construct_algebraic_real_1 construct_algebraic_real_1 = ak_ptr->construct_algebraic_real_1_object();
    return construct_algebraic_real_1(r);
  }
public: //same conversion functions that follow the concept required by Interval's point_in_interval
  Approximated_nt approximate(const Exact_nt& a, bool larger = true)
  {
    return CGAL::to_double(a);
  }
  Approximated_nt approximate_in_interval(const Exact_nt& low, const Exact_nt& high)
  {
    return (CGAL::to_double(low) + CGAL::to_double(high)) / 2;
  }
  Exact_nt convert(const Approximated_nt& r)
  {
    return construct_algebraic(r);
  }

}; //Algebraic_kernel_d_1_conversions_double

#endif  //AK_CONVERSIONS_1_H