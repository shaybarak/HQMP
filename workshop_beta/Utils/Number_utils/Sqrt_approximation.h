#ifndef SQRT_APPROXIMATION_H
#define SQRT_APPROXIMATION_H

#define SQRT_APPROXIMATION_EPSILON      0.0001
#define SQRT_APPROXIMATION_LOWER_BOUND  0.9
#define SQRT_APPROXIMATION_HIGHER_BOUND  1.1

template <typename NT>
class Sqrt_approximation
{
private:
  NT _epsilon;
public:
  Sqrt_approximation(const NT& epsilon = SQRT_APPROXIMATION_EPSILON)
    :_epsilon(epsilon)
  {}
  NT operator() (const NT& value)
  {
    NT rough_approximation = get_rough_approximation(value);
    return successive_approximation(value, rough_approximation);
  }
  NT operator() (const NT& value, bool larger)
  {
    NT rough_approximation = get_rough_approximation(value);
    
    
    if (larger)
    {
      //get a rough approximation of the value guaranteed to be 
      //larger or equal to the correct square root
      while (CGAL::compare (approx*approx, value) == CGAL::SMALLER)
        rough_approximation*=SQRT_APPROXIMATION_HIGHER_BOUND;
    }
    else //smaller
    {
      //get a rough approximation of the value guaranteed to be 
      //smaller or equal to the correct square root
      while (CGAL::compare (approx*approx ,value) == CGAL::LARGER)
        rough_approximation*=SQRT_APPROXIMATION_LOWER_BOUND;
    }
    
    return successive_approximation(value, rough_approximation);
  }

private:
  NT get_rough_approximation (const NT& value)
  {
    double v (CGAL::to_double(value));
    return NT(sqrt(v));
  }

  NT successive_approximation(const NT& value, const NT& rough_approximation)
  {
    //check for valid input value 
    CGAL_precondition (CGAL::sign(value) != CGAL::NEGATIVE);
    if (CGAL::sign(value) == CGAL::ZERO)
      return(value);
   
    //start with some guess
    NT old_approximation  (rough_approximation);
    NT new_approximation  ((old_approximation + (value / old_approximation)) / 2.0); 
    NT delta              ((old_approximation - new_approximation) / new_approximation);

    // start successive approximation//
    while ( (CGAL::compare (delta,  _epsilon) == CGAL::LARGER) ||
            (CGAL::compare (delta, -_epsilon) == CGAL::SMALLER) )
    {
      old_approximation = new_approximation;
      new_approximation = (old_approximation + (value / old_approximation)) / 2.0;
      delta = (old_approximation - new_approximation) / new_approximation;
    } 
    return (new_approximation);
  }
}; //Sqrt_approximation

#endif  //SQRT_APPROXIMATION_H