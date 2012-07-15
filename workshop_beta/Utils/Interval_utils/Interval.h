#ifndef INTERVAL_H
#define INTERVAL_H

template <class NT>
class Default_convertor
{
public:
  typedef NT      Exact_nt;
  typedef NT      Approximated_nt;
public:
  //CTR's, DTR's
  Default_convertor() {}
public: //conversion functions
  Approximated_nt approximate(const Exact_nt& a)
  {
    return a;
  }
  Approximated_nt approximate_in_interval(const Exact_nt& low, const Exact_nt& high)
  {
    return (low + high)/2;
  }
  Exact_nt        convert(const Approximated_nt& a)
  {
    return a;
  }

}; //Default_convertor


//assumption: all intervals are closed
//An interval is templated with two arguments
//  The first represents the number type used for the lower and higher bounds
//  The second represents a possible class approximating the first (in case of Algebraic numbers for instance)
template <class _NT, typename _Convertor = Default_convertor<_NT> >
class Interval
{
public:
  typedef _NT         NT;
  typedef _Convertor  Convertor;
private:
  static const int NO_VALUE = 0;
private:
	NT    _lower, _higher;
	bool  is_lower_infinity, is_higher_infinity;

public: //CTRs
  //unbounded interval 
  Interval()
    :is_lower_infinity(true),
     is_higher_infinity(true), 
     _lower(NO_VALUE), 
     _higher(NO_VALUE)
  {}
  //one side bounded interval
  Interval(const NT& val, bool low_infinity, bool high_infinity)
    :is_lower_infinity(low_infinity),
     is_higher_infinity(high_infinity),
     _lower(low_infinity   ? NO_VALUE : val),
     _higher(high_infinity ? NO_VALUE : val)
	{
      CGAL_precondition(low_infinity != high_infinity);
	}

  //bounded interval
  Interval(const NT& low, const NT& high):
    is_lower_infinity(false), is_higher_infinity(false),
    _lower(low), _higher(high)
    {
      CGAL_precondition(high > low);
    }

public: //boolean queries
  bool is_unbounded() const
  {
    if ((is_lower_infinity == true) &&
        (is_higher_infinity == true))
      return true;
    return false;
  }

  bool is_bounded() const
  {
    if (  (is_lower_infinity == false) &&
    	  (is_higher_infinity == false))
      return true;
    return false;
  }

  bool is_lower_bounded() const
  {
    return (!is_lower_infinity);
  }

  bool is_higher_bounded() const
  {
    return (!is_higher_infinity);
  }
public: 
  const NT& lower() const   { return _lower;}
  const NT& higher() const  { return _higher;}
public:
  void set_lower(const NT& val)
  {
    CGAL_precondition (CGAL::compare (_higher,val) != CGAL::SMALLER);
    is_lower_infinity = false;
    _lower = val;
  }
  void set_higher(const NT& val)
  {
    CGAL_precondition (CGAL::compare (val,_lower) != CGAL::LARGER);
    is_higher_infinity = false;
    _higher = val;
  }
  void set_lower_unbounded()  { is_lower_infinity = true;}  
  void set_higher_unbounded() { is_higher_infinity = true;}
public:
  bool contains(const NT& val) const
  {
    if ((is_lower_infinity == false) && 
        (CGAL::compare(val, _lower) != CGAL::LARGER) )
      return false;
    if ((is_higher_infinity == false) && 
        (CGAL::compare(val, _higher)!= CGAL::SMALLER) )
      return false;
    return true;
  }
  bool contains(const typename Convertor::Approximated_nt& val,
                Convertor& convertor = Convertor()) const
  {
    BOOST_STATIC_ASSERT((boost::is_same<Convertor::Exact_nt, NT>::value));

    if (is_lower_infinity == false)
    {
      Convertor::Approximated_nt lower_bound_on_lower = convertor.approximate(_lower, false);
      if (CGAL::compare(val, lower_bound_on_lower) != CGAL::LARGER) 
        return false;

      Convertor::Approximated_nt upper_bound_on_lower = convertor.approximate(_lower, true);
      if (CGAL::compare(val, upper_bound_on_lower) != CGAL::LARGER) 
        return contains(convertor.convert(val));
    }

    if (is_lower_infinity == false)
    {
      Convertor::Approximated_nt upper_bound_on_higher = convertor.approximate(_higher, true);
      if (CGAL::compare(val, upper_bound_on_higher) != CGAL::SMALLER) 
        return false;

      Convertor::Approximated_nt lower_bound_on_higher = convertor.approximate(_higher, false);
      if (CGAL::compare(val, lower_bound_on_higher) != CGAL::SMALLER) 
        return contains(convertor.convert(val));
    }

    return true;
  }

  typename Convertor::Approximated_nt point_in_interval(Convertor& convertor = Convertor()) const
  {
    BOOST_STATIC_ASSERT((boost::is_same<Convertor::Exact_nt, NT>::value));

    typedef Convertor::Approximated_nt NT2;
    
    if (is_lower_infinity  && is_higher_infinity)
      return NT2(0);

    NT2 step(1);
    NT2 res(0);
    if (is_lower_infinity) 
    {
      if (CGAL::sign(_higher) == CGAL::POSITIVE)
        return NT2(0);
      else
        res = convertor.approximate(_higher,false) - step;
    }

    else if (is_higher_infinity)
    {
      if (CGAL::sign(_lower) == CGAL::NEGATIVE)
        return NT2(0);
      else
        res = convertor.approximate(_lower,true) + step;
    }

    else // (!is_lower_infinity && !is_higher_infinity)
    {
      res = convertor.approximate_in_interval(_lower,_higher);
    }

    //try to simplify
    if (contains(NT(int(CGAL::to_double(res)))))
      res = int(CGAL::to_double(res));      
    else if (contains(NT(CGAL::to_double(res))))
      res = CGAL::to_double(res);
    
    return res;
  }
public: //IO
  void print_nice() const
  {
    cout << "[ " ;
    if (is_lower_infinity == true)
      cout << "-infinity" ;
    else
      cout << CGAL::to_double(_lower);

    cout << " --> ";

    if (is_higher_infinity == true)
      cout << "infinity" ;
    else
      cout << CGAL::to_double(_higher);

    cout << " ]   ";
  }


};



#endif // INTERVAL_H