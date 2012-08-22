#ifndef REFERENCE_POINT_T_H
#define REFERENCE_POINT_T_H

#include "Utils\Rotation_utils\Rotation.h"

template <typename _K>
class Reference_point
{
public:
	typedef _K                                K;
	typedef typename K::FT					  FT;
	typedef typename K::Point_2               Point;
	typedef typename Rotation<typename K::FT> Rotation;

	// Creates a new reference point that is between two other reference points given a ratio.
	// Returns a reference point with coordinates such that x == x_1 * ratio + x_2 * (1-ratio),
	// and similarly for y and theta.
	static Reference_point get_point_between(const Reference_point& point1,
		const Reference_point& point2, double ratio) {
			FT x = point1.get_location().x() * ratio
				+ point2.get_location().x() * (1 - ratio);
			FT y = point1.get_location().y() * ratio
				+ point2.get_location().y() * (1 - ratio);
			double angle = point1.get_rotation().to_angle(RAD) * ratio
				+ point2.get_rotation().to_angle(RAD) * (1 - ratio);
			return Reference_point(Point(x, y), to_rotation<FT>(angle, RAD));
	}
	//constructors
	Reference_point(){}
	Reference_point(Point p, Rotation r)
		: _p(p), _r(r)
	{
		CGAL_precondition(r.has_complete_rotation() == false);
	}
	Reference_point(const Reference_point<K>& other) 
		: _p(other.get_location()), _r(other.get_rotation()) {
	}

	//assignment operator
	Reference_point<K>& operator= (const Reference_point<K> &other)
	{
		if (this != &other) 
		{
			_p = other.get_location();
			_r = other.get_rotation();
		}
		return *this;
	}

	bool operator== (const Reference_point<K> &other) const
	{
		return ((_p == other.get_location()) &&
			(_r == other.get_rotation()) );
	}
	bool operator!= (const Reference_point<K> &other) const
	{
		return ((_p.x() != other.get_location().x()) ||
			(_p.y() != other.get_location().y()) ||
			(_r != other.get_rotation()) );
	}
	//get
	Point get_location() const
	{
		return _p;
	}
	const Rotation& get_rotation() const
	{
		return _r;
	}

	//set
	void set_location(const Point& p)
	{
		_p = p;
		return;
	}
	void set_rotation(const Rotation& r)
	{
		CGAL_precondition(r.has_complete_rotation() == false);
		_r = r;
		return;
	}

	//print
	void print() const
	{
		std::cout << "[";
		print_point_nice<K>(_p);
		std::cout << " , ";
		_r.print_nice();
		std::cout << "]";
	}
	void write(std::ostream& os)
	{
		os  <<  _p.x() << std::endl
			<<  _p.y() << std::endl
			<<  _r.sin() << std::endl
			<<  _r.cos() << std::endl;
		return;
	}
	void read(std::istream& is)
	{
#if EXACT_READ_MODE	
		typename K::FT  x, y;
		is >> x >> y;
		_p = Point(x, y);

		typename K::FT  sin, cos;
		is >> sin >> cos;
		_r = Rotation(sin, cos);
#else //EXACT_READ_MODE
		double  x, y;
		is >> x >> y;
		_p = Point(x, y);

		double angle;
		is >>angle;
		_r = to_rotation<typename K::FT> (angle, DEG);
#endif //EXACT_READ_MODE
		return;
	}

	//Methods for easy computation distance and time betwwen points, with no need for verbose motion_step

	double get_angular_delta(const Reference_point& other) const{
		double delta = abs(get_rotation().to_angle() - other.get_rotation().to_angle());
		if (delta > 180) {
			delta = (360 - delta);
		}
		return delta;
	}

	double get_translational_delta(const Reference_point& other) const{
		Sqrt_approximation<typename K::FT> sqrt_approximation;
		K::FT d_sq = CGAL::squared_distance(get_location(), other.get_location());
		double delta = CGAL::to_double(sqrt_approximation(d_sq));
		return delta;
	}

	double get_angular_time(const Reference_point& other) const{
		return get_angular_delta(other) / configuration.get_rotational_speed();
	}

	double get_translational_time(const Reference_point& other) const{
		return get_translational_delta(other) / configuration.get_translational_speed();
	}

	double get_aerial_time(const Reference_point& other) const {
		double time = get_angular_time(other) + get_translational_time(other);
		return time;
	}


private:
	Point    _p;    //location
	Rotation _r;    //rotation
}; //Reference_point


template <typename K>
struct Less_than_reference_point
{
	bool operator()(const Reference_point<K>& p1,const Reference_point<K>& p2) const 
	{
		CGAL::Comparison_result cr = CGAL::compare_xy(p1.get_location(),
			p2.get_location());
		if (cr != CGAL::EQUAL)
			return (cr == CGAL::SMALLER) ? true : false;

		//(cr == CGAL::EQUAL)
		const Rotation<typename K::FT>& r1 = p1.get_rotation();
		const Rotation<typename K::FT>& r2 = p2.get_rotation();
		return (( r1.is_larger_than(r2) || (r1 == r2)) ?
			false:
		true);
	}
}; //Less_than_reference_point
#endif REFERENCE_POINT_T_H