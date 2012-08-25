#include "stdafx.h"
#include "MyPlayer.h"

MyPlayer::MyPlayer(Env* env, Configuration* config) :
	Player(env, config),
	planner(env->get_workspace(),
			env->get_robot_a()),
	pending_motion_end(env->get_source_configuration_a()),
	planner_initialized(false),
	query_again(true) {
}

// Spend as much time preprocessing and planning
bool MyPlayer::plan(double deadline) {
	TIMED_TRACE_ENTER("plan");
	initialize();

	// If last attempt to plan a path succeeded and there are remaining targets
	if (query_again && !env->get_target_configurations().empty()) {
		// Plan another motion step
		query_again = buffer_motion_ahead(pending_motion_end);
		// TODO if querying succeeded but only found a path to one remaining target, the next call will be a waste of time
		// Consider adding a planner method that checks connectivity and use that for making decisions
	} else {
		// Spend some time doing additional preprocessing
		// TODO different handling for additional preprocessing when path not found and when no targets remain
		// TODO consider renaming this method to enhance_connectivity
		planner.additional_preprocessing(pending_motion_end, env->get_target_configurations());
		// Try again on the next call to plan
		query_again = true;
	}
	
	TIMED_TRACE_EXIT("plan");
	return true;
}

// Generate next movement
bool MyPlayer::move(double deadline, Motion& motion_sequence) {
	initialize();
	CGAL::Timer timer;
	timer.start();

	if (pending_motion.empty()) {
		// No pending motion, buffer some more
		TIMED_TRACE_ACTION("move", "no pending movement");
		if (!plan(deadline)) {
			// Could not generate additional motion at this time, skip this turn
			TIMED_TRACE_ACTION("move", "could not find additional motion");
			return false;
		} else {
			TIMED_TRACE_ACTION("move", "additional motion found");
		}
	}
	
	// Extract longest possible motion out of pending motion (under deadline)
	pending_motion.cut_motion(deadline - timer.time(), motion_sequence);
	return true;
}

// Returns whether the player thinks that the game is over.
bool MyPlayer::is_game_over() {
	return (
		// No additional targets
		env->get_target_configurations().empty() &&
		// No planned pending motion
		pending_motion.empty());
}

// Initializes the underlying planner.
// Returns true iff the planner was initialized on this invocation.
bool MyPlayer::initialize() {
	if (planner_initialized) {
		return false;
	}

	// Allow additional preprocessing to source point and all initial targets
	Ref_p_vec additional_samples = env->get_target_configurations();
	additional_samples.push_back(env->get_source_configuration_a());
	planner.preprocess_targets(additional_samples);

	planner.preprocess();

	// TODO: remove, only here for debugging
	additional_samples.clear();
	additional_samples = env->get_additional_sample_points();
	if (!additional_samples.empty()) {
		planner.preprocess_targets(env->get_additional_sample_points());
	}

	planner_initialized = true;
	return true;
}

// Buffers additional motion, inserting it to the back of the pending motion.
// Returns whether additional motion could be buffered.
bool MyPlayer::buffer_motion_ahead(const Reference_point& source) {
	TIMED_TRACE_ENTER("buffer_motion_ahead");
	CGAL_precondition(!env->get_target_configurations().empty());
	Ref_p_vec::iterator next_target;
	int target_index;

	// Plan a motion to the closest remaining target
	bool path_found = planner.query_closest_point(
		source, 
		env->get_target_configurations(),
		target_index,
		// Append it to the buffered motion plan
		pending_motion);
	
	if (path_found) {
		// Reached another target
		next_target = env->get_target_configurations().begin() + target_index;
		pending_motion_end = *next_target;
		env->get_target_configurations().erase(next_target);
	} else {
		TIMED_TRACE_ACTION("buffer_motion_ahead", "path not found");
	}
	
	TIMED_TRACE_EXIT("buffer_motion_ahead");
	return path_found;
}

void MyPlayer::additional_targets_preprocessing(Ref_p_vec& additional_targets) {
	planner.preprocess_targets(additional_targets);
}
