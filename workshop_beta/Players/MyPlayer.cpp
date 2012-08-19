#include "stdafx.h"
#include "MyPlayer.h"

MyPlayer::MyPlayer(Env* env, Configuration* config) :
	Player(env, config),
	planner(env->get_workspace(),
	env->get_robot_a()),
	pending_motion_end(env->get_source_configuration_a()) {
}

// Spend as much time preprocessing and planning
void MyPlayer::plan(double deadline) {
	TIMED_TRACE_ENTER("plan");
	initialize();
	CGAL::Timer timer;
	timer.start();

	// TODO use the extra time for additional planner preprocessing?

	// Plan additional future motion steps until time is up
	while (deadline - timer.time() > 0) {
		if (!buffer_motion_ahead()) {
			// Stop when no additional planning is possible
			break;
		}
	}
	TIMED_TRACE_EXIT("plan");
}

// Generate next movement
void MyPlayer::move(double deadline, Motion& motion_sequence) {
	initialize();
	CGAL::Timer timer;
	timer.start();

	if (pending_motion.empty()) {
		// No pending motion, buffer some more
		if (!buffer_motion_ahead()) {
			// Could not generate additional motion at this time, skip this turn
			return;
		}
	}
	
	// Extract longest possible motion out of pending motion (under deadline)
	pending_motion.cut(deadline - timer.time(), motion_sequence);
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
	if (planner.initialized()) {
		return false;
	}
	planner.preprocess();
	return true;
}

// Buffers additional motion, inserting it to the back of the pending motion.
// Returns whether additional motion could be buffered.
bool MyPlayer::buffer_motion_ahead() {
	TIMED_TRACE_ENTER("buffer_motion_ahead");
	CGAL_precondition(!env->get_target_configurations().empty());
	Ref_p_vec::iterator next_target;

	// Plan a motion to the closest remaining target
	bool path_found = planner.query_closest_point(
		pending_motion_end, 
		env->get_target_configurations(),
		next_target,
		// Append it to the buffered motion plan
		pending_motion);

	if (!path_found) {
		TIMED_TRACE_ACTION("query", "path not found");
		TIMED_TRACE_EXIT("buffer_motion_ahead");
		return false;
	}
	
	env->get_target_configurations().erase(next_target);
	pending_motion_end = *next_target;
	TIMED_TRACE_EXIT("buffer_motion_ahead");
	return true;
}