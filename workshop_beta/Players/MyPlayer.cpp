#include "stdafx.h"
#include "MyPlayer.h"

MyPlayer::MyPlayer(Env* env, Configuration* config) :
	Player(env, config),
	// Initialize planner
	planner(env->get_workspace(), env->get_robot_a()),
	// End of buffer is the source configuration
	buffer_end(env->get_source_configuration_a()),
	// Remember to initialize the planner
	planner_initialized(false),
	last_query_succeeded(true) {
}

// Perform preprocessing and buffering of future motion
bool MyPlayer::plan(double deadline) {
	TIMED_TRACE_ENTER("plan");
	initialize();

	// If last attempt to plan a path succeeded and there are remaining targets
	if (last_query_succeeded && !env->get_target_configurations().empty()) {
		// Plan additional motion to the next target
		Motion new_motion;
		last_query_succeeded = move_to_closest_target(buffer_end, env->get_target_configurations(), new_motion);
		motion_buffer.push_back(new_motion);
		buffered_targets.push_back(buffer_end);
		return true;
	} else {
		// Spend some time doing additional preprocessing
		planner.additional_preprocessing(buffer_end, env->get_target_configurations());
		// Try again on the next call to plan
		last_query_succeeded = true;
	}
	
	TIMED_TRACE_EXIT("plan");
	return true;
}

// Generate next movement
bool MyPlayer::move(double deadline, Motion& motion_output) {
	initialize();
	CGAL::Timer timer;
	timer.start();

	if (motion_buffer.empty()) {
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

	// Attempt to perform the first motion in the buffer
	Motion& motion = motion_buffer.front();
	MS_base* step = NULL;
	bool blocked = false;
	
	while (!motion.empty() && !blocked && (deadline - timer.time() > 0)) {
		// Get the next step
		motion.cut_step(deadline - timer.time(), motion_output);
		step = motion_output.back();
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
			// Return it to the buffer
			motion.add_motion_step_front(step);
			// Don't output this invalid step
			motion_output.pop_back();
			// TODO find the entire gap and find a path around it
			// Right now we can't do anything useful with our remaining time
			blocked = true;
			break;
		}
	}

	if (step == NULL) {
		return false;
	}
	if (motion.empty()) {
		// Executed the entire motion
		buffered_targets.pop_front();
		motion_buffer.pop_front();
	}
	return !blocked;
}

// Returns whether the player thinks that the game is over.
bool MyPlayer::is_game_over() {
	return (
		// No target configurations left
		env->get_target_configurations().empty() &&
		// No buffered targets left
		buffered_targets.empty());
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

// Moves to the closest target (appending the motion).
// Returns whether a motion was successfully found.
// Updates source to the target selected.
// Removes said target from targets.
bool MyPlayer::move_to_closest_target(Reference_point& source, Reference_point_vec& targets, Motion& motion) {
	TIMED_TRACE_ENTER("buffer_motion_ahead");
	CGAL_precondition(!env->get_target_configurations().empty());
	Ref_p_vec::iterator target_reached;
	int target_index;

	// Verify that at least one target is reachable
	if (!planner.exist_reachable_target(source, targets)) {
		return false;
	}

	// Plan a motion to the closest remaining target
	bool path_found = planner.query_closest_point(
		source, 
		targets,
		target_index,
		// Append it to the output motion
		motion);
	ASSERT_CONDITION(path_found, "targets are connected but could not find path!");
	
	if (path_found) {
		// Reached another target
		target_reached = targets.begin() + target_index;
		source = *target_reached;
		targets.erase(target_reached);
	} else {
		TIMED_TRACE_ACTION("buffer_motion_ahead", "path not found");
	}
	
	TIMED_TRACE_EXIT("buffer_motion_ahead");
	return true;
}

void MyPlayer::additional_targets_preprocessing(Ref_p_vec& additional_targets) {
	planner.preprocess_targets(additional_targets);
}
