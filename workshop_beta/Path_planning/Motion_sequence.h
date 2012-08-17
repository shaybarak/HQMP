#ifndef MOTION_SEQUENCE_H_
#define MOTION_SEQUENCE_H_

#include "Utils\ReferencePoint.h"
#include "Path_planning\Motion_step_base.h"
#include "Path_planning\Motion_step_translational.h"
#include "Path_planning\Motion_step_rotational.h"
#include "Path_planning\Motion_step_static.h"

#include "Path_planning\Motion_step_writer.h"
#include "Path_planning\Motion_step_reader.h"

template <typename K>
class Motion_sequence
{
public:
	typedef Motion_step_base<K>					MS_base;
	typedef Motion_sequence<K>					Self;
	typedef Motion_step_translational<K>		MS_translational;
	typedef Motion_step_rotational<K>			MS_rotational;
	typedef Motion_step_static<K>				MS_static;
	typedef MS_base*							MS_base_ptr;
	typedef std::deque<typename MS_base_ptr>	Motion;
	typedef Reference_point<K>					Ref_point;
	typedef CGAL::Quotient<K>					Quotient;

	Motion_sequence() {}

	~Motion_sequence() 
	{
		clear();
	}

	void clear()
	{
		BOOST_FOREACH(MS_base_ptr motion_step_ptr, motion_sequence)
			delete motion_step_ptr;
		motion_sequence.clear();
	}

	template <typename InputIterator>
	Motion_sequence(InputIterator& begin, InputIterator& end) 
	{
		for (InputIterator iter = begin; iter != end; ++iter)
			add_motion_step(*iter);    
	}

	void add_motion_step(const MS_base_ptr& motion_step)
	{
		CGAL_precondition ( (motion_sequence.empty()) || 
			(motion_sequence.back()->target() == motion_step->source()) );

		motion_sequence.push_back(motion_step);
		return;
	}

	void add_motion_sequence(const Self& other_motion_sequence)
	{
		const Motion& other_motion = other_motion_sequence.get_sequence();
		BOOST_FOREACH(MS_base_ptr motion_step_ptr, other_motion)
		{
			//ptr needs to be copied as each Motion_sequence class is in charge of deleting it's own MS_base_ptr
			CGAL_precondition (motion_step_ptr->type() != MS_base::UNINITIALIZED);
			switch (motion_step_ptr->type())
			{
			case (MS_base::TRANSLATION) : 
				{
					MS_translational* motion_step_ptr_copy = new MS_translational(* static_cast<MS_translational*> (motion_step_ptr));
					add_motion_step(motion_step_ptr_copy);
					break;
				}
			case (MS_base::ROTATION):
				{
					MS_rotational* motion_step_ptr_copy = new MS_rotational(* static_cast<MS_rotational*> (motion_step_ptr));
					add_motion_step(motion_step_ptr_copy);
					break;
				}
			case (MS_base::STATIC):
				{
					MS_static* motion_step_ptr_copy = new MS_static(* static_cast<MS_static*> (motion_step_ptr));
					add_motion_step(motion_step_ptr_copy);
					break;
				}
			}
		}
		return;
	}

	const Motion& get_sequence() const
	{
		return motion_sequence;
	}

	void reverse_motion_sequence()
	{
		BOOST_FOREACH (MS_base_ptr motion_step, motion_sequence)
			motion_step->reverse_motion_step();

		std::reverse(motion_sequence.begin(), motion_sequence.end());
	}

	

	double motion_time_between(int first_index, int last_index) {
		return motion_time_between(
			first_index,
			last_index,
			configuration.get_translational_speed(),
			configuration.get_rotational_speed());
	}

	double motion_time() {
		return motion_time_between(0, motion_sequence.size()-1, 
			configuration.get_translational_speed(), configuration.get_rotational_speed());   
	}

	double motion_time(const double translational_speed, const double rotational_speed) {
		return motion_time_between(0, motion_sequence.size()-1, 
			translational_speed, rotational_speed);   
	}


	//compute motion time between indices first and last, including both first and last
	double motion_time_between(
		int first_index,
		int last_index,
		const double translational_speed,
		const double rotational_speed) {
		
		double t = 0;
		
		for (int i=first_index; i<=last_index; i++) {
			t += step_time(motion_sequence[i], translational_speed, rotational_speed);
		}
		return t;
	}

	static double step_time(MS_base_ptr& motion_step_ptr) {
		return step_time(
			motion_step_ptr, 
			configuration.get_translational_speed(), 
			configuration.get_rotational_speed());
	}

	static double step_time( MS_base_ptr& motion_step_ptr,
		const double translational_speed, //unit per second
		const double rotational_speed) //full rotation per second
	{
		double t(0);
		if (motion_step_ptr->type() == MS_base::TRANSLATION)
			t+=motion_step_ptr->approximate_motion_time(translational_speed);
		else if (motion_step_ptr->type() == MS_base::ROTATION)
			t+=motion_step_ptr->approximate_motion_time(rotational_speed);
		return t;
	}

	void read(std::istream& is)
	{
		int n;    
		is >> n;

		Motion_step_reader<K> reader(is);
		for (int i(0); i<n; ++i)
			add_motion_step(reader());

		return;
	}

	void write(std::ostream& os)
	{
		if (motion_sequence.empty())
			return;

		os << motion_sequence.size() <<std::endl;
		Motion_step_writer<K> writer(os);
		BOOST_FOREACH(MS_base_ptr motion_step_ptr, motion_sequence)
			writer(motion_step_ptr);

		return;
	}

	// Given a movement time, returns the part of the motion sequence that can be performed within this time.
	// Motion steps that were cut are returned to the caller and removed from the original sequence.
	// This method may break down a single step into two steps to fit part of it into the new motion.
	void cut(double remaining_time, // Time left in seconds
		const double translational_speed, // Unit per second
		const double rotational_speed, // Full rotation per second
		Motion_sequence& output) {
			while ((remaining_time > 0) && !motion_sequence.empty()) {
				MS_base_ptr next_step = motion_sequence.front();
				double next_step_time = step_time(next_step, translational_speed, rotational_speed);

				if (remaining_time > next_step_time) {
					// Add the step to the cut
					output.motion_sequence.push_back(next_step);
					motion_sequence.pop_front();

				} else {
					// Cut the step according to the remaining time
					Ref_point source = next_step->source();
					Ref_point target = next_step->target();
					double ratio = remaining_time / next_step_time;
					Ref_point between_point = Ref_point::get_point_between(source, target, ratio);
					MS_base_ptr cut_step, remaining_step;
					if (next_step->type() == MS_base::TRANSLATION) {
						cut_step = new MS_translational(source, between_point);
						remaining_step = new MS_translational(between_point, target);
					} else if (next_step->type() == MS_base::ROTATION) {
						CGAL::Orientation orientation = static_cast<MS_rotational*>(next_step)->orientation();
						cut_step = new MS_rotational(source, between_point, orientation);
						remaining_step = new MS_rotational(between_point, target, orientation);
					}
					// Add the part of the step that can be made in the remaining time
					output.motion_sequence.push_back(cut_step);
					// Return the rest of the step to the remaining motion sequence
					motion_sequence.pop_front();
					motion_sequence.push_front(remaining_step);
				}

				remaining_time -= next_step_time;
			}
	}

	// Whether this motion sequence is empty
	bool empty() {
		return motion_sequence.empty();
	}

	void cut(double remaining_time, Motion_sequence& output) {
		cut(remaining_time, configuration.get_translational_speed(), configuration.get_rotational_speed(), output);
	}

private:
	Motion motion_sequence;
}; //Motion_sequence
#endif //MOTION_SEQUENCE_H_