#include "stdafx.h"
#include "FastCopyingPlayer.h"

FastCopyingPlayer::FastCopyingPlayer(Env* env, Configuration* config) :
	Player(env, config),
	// Initialize planner
	original_planner(env->get_workspace(), env->get_robot_a()),
	// Don't initialize cloned planner yet
	cloned_planner(NULL),
	// Initial location is the source configuration
	location(env->get_source_configuration_a()),
	// Remember to initialize the planner
	planner_initialized(false),
	is_last_motion_complete(false) {
}

FastCopyingPlayer::~FastCopyingPlayer() {
	if (cloned_planner != NULL) {
		delete cloned_planner;
	}
}

// Perform preprocessing and buffering of future motion
bool FastCopyingPlayer::plan(double deadline) {
	TIMED_TRACE_ENTER("plan");
	ASSERT_CONDITION(deadline > 0, "plan called but no time left");
	initialize();
	timed_message("Thinking...");
	if (original_planner.preprocess_plan()) {
		TIMED_TRACE_EXIT("plan: can do more preprocessing");
		return true;
	} else {
		TIMED_TRACE_EXIT("plan: no more preprocessing please");
		timed_message("I'm tired of thinking.");
		return false;
	}
}

// Generate next movement
bool FastCopyingPlayer::move(double deadline, Motion& motion_output) {
	CGAL::Timer timer;
	timer.start();
	ASSERT_CONDITION(deadline > 0, "move called but no time left");
	timed_message("Planning a move...");
	initialize();

	sample_current_location();

	// Make sure we have at least one reachable target before proceeding
	while ((timer.time() < deadline) && !has_reachable_targets(original_planner)) {
		timed_message("Thinking harder...");
		TIMED_TRACE_ACTION("move", "preprocessing_move original planner");
		original_planner.preprocess_move();
	}
	if (timer.time() >= deadline) {
		return false;
	}

	// Create a copy of the planner, introducing the dynamic obstacle's current configuration
	clone_planner();
	// Verify that the dynamic obstacle does not block all remaining targets
	if (!has_unblocked_targets(*cloned_planner)) {
		timed_message("Arrgh! The other robot is sitting on top of the target!");
		return false;
	}

	// Now do the same thing with the clone
	while ((timer.time() < deadline) && !has_reachable_targets(*cloned_planner)) {
		timed_message("Thinking even harder...");
		TIMED_TRACE_ACTION("move", "preprocessing_move cloned planner");
		cloned_planner->preprocess_move();
	}
	if (timer.time() >= deadline) {
		return false;
	}

	// Plan a motion to the closest target
	timed_message("Making my way to a target...");
	Motion new_motion;
	Ref_p_vec::iterator target_iter = move_to_closest_target(
		*cloned_planner,
		location,
		remaining_targets(),
		new_motion);

	// Check output motion time
	if (new_motion.motion_time() < (deadline - timer.time())) {
		// Output the entire motion
		motion_output.add_motion_sequence(new_motion);
		// Update location to target reached
		location = *target_iter;
		is_last_motion_complete = true;
		last_target = *target_iter;
		// Remove target reached from targets left
		remaining_targets().erase(target_iter);
		timed_message("Reached a target!");
	} else {
		// Cut the motion
		new_motion.cut_motion(deadline - timer.time(), motion_output);
		// Update location to end of new motion
		if (!new_motion.empty()) {
			location = new_motion.front()->source();
		}
		is_last_motion_complete = false;
		timed_message("Moving to target but couldn't reach within time limit.");
	}
	return true;
}

// Returns whether the player thinks that the game is over.
bool FastCopyingPlayer::is_game_over() {
	if (!has_remaining_targets()) {
		timed_message("*** REACHED ALL TARGETS! ***");
		return true;
	} else {
		return false;
	}
}

void FastCopyingPlayer::additional_targets_preprocessing(Ref_p_vec& additional_targets) {
	original_planner.preprocess_targets(additional_targets);
}

void FastCopyingPlayer::reject_last_move(Motion& motion_sequence) {
	// If the last (rejected) motion was to a target
	if (is_last_motion_complete) {
		// Return that target to the list of remaining targets
		remaining_targets().push_back(last_target);
	}
}

// Initializes the underlying planner.
// Returns true iff the planner was initialized on this invocation.
bool FastCopyingPlayer::initialize() {
	if (planner_initialized) {
		return false;
	}
	original_planner.initialize(location, remaining_targets());
	planner_initialized = true;
	return true;
}

void FastCopyingPlayer::sample_current_location() {
	Ref_p_vec source_vec;
	source_vec.push_back(location);
	original_planner.preprocess_targets(source_vec);
}

void FastCopyingPlayer::clone_planner() {
	if (cloned_planner != NULL) {
		if (dynamic_obstacle == cached_dynamic_obstacle) {
			// No need to re-clone the planner!
			return;
		} else {
			// The cloned planner is obsolete
			delete cloned_planner;
		}
	}

	Extended_polygon robot_b(env->get_robot_b());
	robot_b.move_absolute(dynamic_obstacle);
	cloned_planner = new Planner(original_planner, robot_b);
	cached_dynamic_obstacle = dynamic_obstacle;
}

// Moves to the closest target
Ref_p_vec::iterator FastCopyingPlayer::move_to_closest_target(Planner& planner, Reference_point& source, Ref_p_vec& targets, Motion& motion) {
	Ref_p_vec::iterator target_reached;
	int target_index;

	// Plan a motion to the closest remaining target
	bool path_found = planner.query_closest_point(
		source, 
		targets,
		target_index,
		// Append it to the output motion
		motion);
	ASSERT_CONDITION(path_found, "targets are connected but could not find path!");

	return targets.begin() + target_index;
}

// Are there any remaining targets?
bool FastCopyingPlayer::has_remaining_targets() {
	return !remaining_targets().empty();
}

// Are there remaining targets that are reachable?
bool FastCopyingPlayer::has_reachable_targets(Planner& planner) {
	return (!remaining_targets().empty()
		&& planner.exist_reachable_target(location, remaining_targets()));
}

// Are there remaining targets that are unreachable?
bool FastCopyingPlayer::has_unreachable_targets(Planner& planner) {
	return (!remaining_targets().empty()
		&& !planner.exist_unreachable_target(location, remaining_targets()));
}

// Are there remaining targets that are non-blocked?
bool FastCopyingPlayer::has_unblocked_targets(Planner& planner) {
	for (Ref_p_vec::iterator iter = remaining_targets().begin(); iter != remaining_targets().end(); iter++) {
		if (planner.is_free(*iter)) {
			return true;
		}
	}
	return false;
}

Ref_p_vec& FastCopyingPlayer::remaining_targets() {
	return env->get_target_configurations();
}
