#ifndef INPUT_READER_T_H
#define INPUT_READER_T_H

#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_set_2.h>

#include "Project\Configuration.h"

class Input_reader
{
public:
	//ctr's
	Input_reader(){}
  //configuration
  void read_configuration(const std::string& filename, Configuration& configuration);
  //polygons
  template <typename K> 
  void read_polygon  (const std::string& filename, CGAL::Polygon_2<K>& polygon)
  {
    std::ifstream file (filename.c_str());
    CGAL_precondition(file.is_open());

    read_polygon(file, polygon);

    file.close();
    return;
  }
  template <typename K> 
  void read_polygon  (std::ifstream& is, CGAL::Polygon_2<K>& polygon)
  {
    CGAL_precondition(is.is_open());
      
    int num_of_vertices;
      
    is >> num_of_vertices;
    for (int i = 0 ; i < num_of_vertices; ++i)
    {
      std::string line;
      std::vector<std::string> results;
      getline (is, line);
      if (line == "")
      {
        --i;
        continue;
      }
      string_split(line," ",results);
      K::Point_2 p(to_number_type<K::FT> (results[0]),to_number_type<K::FT>(results[1]));
      polygon.push_back(p);
    }
    if (polygon.is_clockwise_oriented () )
      polygon.reverse_orientation();
    return;
  }
    
  template <typename K> 
  void read_polygons (const std::string& filename, std::vector<CGAL::Polygon_2<K> >& polygons)
  {
    std::ifstream file (filename.c_str());
    CGAL_precondition(file.is_open());

    int num_of_polygons;
    file >> num_of_polygons;

    for (int i(0); i < num_of_polygons; ++i)
    {
      CGAL::Polygon_2<K> polygon;
      read_polygon(file, polygon);
      polygons.push_back(polygon);
    }

    file.close();
    return;
  }

  template <typename K> 
  Reference_point<K> read_reference_point (const std::string& filename)
  {
    ifstream is(filename.c_str());
    CGAL_postcondition(is.is_open());

    Reference_point<K> reference_point;
    reference_point.read(is);

    is.close();

    return reference_point;      
  }
  template <typename K, typename OutputIterator>
  void read_reference_points (const std::string& file_name, OutputIterator& oi)
  {
    ifstream is(file_name.c_str());
    CGAL_postcondition(is.is_open());

    int n;    
    is >> n;  //number of reference points

    for (int i(0); i <n; ++i)
    {
      Reference_point<K> reference_point;
      reference_point.read(is);
      *oi++ = reference_point;
    }

    is.close();
    return;
  }
private:
	void string_split(std::string str, const std::string delim, std::vector<std::string>& results);
  void remove_delimiter(const std::string&  in_file_name, const std::string&  out_file_name, const std::string&  delim);
    
  CGAL::Gmpq  to_number_type_gmpq (std::string input);
	CGAL::CORE_algebraic_number_traits::Rational  to_number_type_core_algebraic (std::string input);
  CGAL::CORE_arithmetic_kernel::Rational    to_number_type_core_arithmetic(std::string input);
  template <typename NT> NT to_number_type_default(std::string input)
    {
      double tmp ( atof(input.c_str()) );
      NT result(tmp);
	  return result;
    }
  template <typename NT> NT to_number_type(std::string input)
  {
    if (boost::is_same<CGAL::Gmpq,NT>::value)
    {
      typedef CGAL::Gmpq NT1;
      CGAL::Cast_function_object<NT1,NT> caster;
      NT1 val (to_number_type_gmpq(input));
      return caster(val);
    }
    else if (boost::is_same<CGAL::CORE_arithmetic_kernel::Rational,NT>::value)
    {
      typedef CGAL::CORE_arithmetic_kernel::Rational NT1;
      CGAL::Cast_function_object<NT1,NT> caster;
      NT1 val (to_number_type_core_arithmetic(input));
      return caster(val);
    }
    else if (boost::is_same<CGAL::CORE_algebraic_number_traits::Rational,NT>::value)
    {
      typedef CGAL::CORE_algebraic_number_traits::Rational NT1;
      CGAL::Cast_function_object<NT1,NT> caster;
      NT1 val (to_number_type_core_algebraic(input));
      return caster(val);
    }
    else 
      return to_number_type_default<NT>(input);
  }

};

#endif  //INPUT_READER_T_H