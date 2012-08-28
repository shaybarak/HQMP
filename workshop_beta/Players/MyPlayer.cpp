#include "stdafx.h"
#include "MyPlayer.h"

MyPlayer::MyPlayer(Env* env, Configuration* config) :
	Player(env, config),
	planner(env->get_workspace(),
			env->get_robot_a()),
	pending_motion_end(env->get_source_configuration_a()),
	planner_initialized(false),
	last_query_succeeded(true) {
}

// Spend as much time preprocessing and planning
bool MyPlayer::plan(double deadline) {
	TIMED_TRACE_ENTER("plan");
	initialize();

	// If last attempt to plan a path succeeded and there are remaining targets
	if (last_query_succeeded && !env->get_target_configurations().empty()) {
		// Plan another motion step
		last_query_succeeded = buffer_motion_ahead(pending_motion_end);
		// TODO if querying succeeded but only found a path to one remaining target, the next call will be a waste of time
		// Consider adding a planner method that checks connectivity and use that for making decisions
		return true;
	} else {
		// Spend some time doing additional preprocessing
		planner.additional_preprocessing(pending_motion_end, env->get_target_configurations());
		// Try again on the next call to plan
		last_query_succeeded = true;
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

	bool step_was_cut = false;
	while (!step_was_cut && (deadline - timer.time() > 0)) {
		// Get the next step
		step_was_cut = pending_motion.cut_step(deadline - timer.time(), motion_sequence);
		MS_base* step = motion_sequence.back();
		deadline -= Motion::step_time(step);
		
		// Validate it
		Extended_polygon robot_a = env->get_robot_a();
		robot_a.move_absolute(step->source());
		Extended_polygon robot_b = env->get_robot_b();
		robot_b.move_absolute(dynamic_obstacle);
		VALIDATION vResult = planner.validate_step(*step, robot_a, robot_b);
		switch (vResult) {
		case OK:
			// Step is valid
			continue;
		case PATH_BLOCKED:
			// Path is blocked but destination is free
		case DST_BLOCKED:
			// Destination is blocked
			// Reject the remaining motion
			// TODO find the entire gap and find a path around it
			pending_motion.add_motion_step_front(step);
			motion_sequence.pop_back();
			// Right now we can't do anything useful with our remaining time
			return false;
		}
	}
	
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

	// Verify that at least one target is reachable
	if (!planner.exist_reachable_target(source, env->get_target_configurations())) {
		return false;
	}

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
