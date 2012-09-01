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

	// Improve connectivity
	if (has_unreachable_targets(original_planner)) {
		TIMED_TRACE_ACTION("plan", "found unreachable targets, improving connectivity");
		improve_connectivity(original_planner, location, env->get_target_configurations());
	}

	// Improve quality
	else {
		// TODO: sample more layers?
		return false;
	}

	// TODO: when do we stop planning?
	return true;
}

// Generate next movement
bool FastCopyingPlayer::move(double deadline, Motion& motion_output) {
	CGAL::Timer timer;
	timer.start();
	initialize();

	// Make sure we have at least one reachable target before proceeding
	while ((timer.time() < deadline) && !has_reachable_targets(original_planner)) {
		TIMED_TRACE_ACTION("move", "trying to connect targets");
		improve_connectivity(original_planner, location, env->get_target_configurations());
	}
	if (timer.time() >= deadline) {
		return false;
	}

	clone_planner();
	// Now do the same thing with the clone
	while ((timer.time() < deadline) && !has_reachable_targets(*cloned_planner)) {
		TIMED_TRACE_ACTION("move", "trying to connect targets");
		improve_connectivity(*cloned_planner, location, env->get_target_configurations());
	}
	if (timer.time() >= deadline) {
		return false;
	}

	// Plan a motion to the closest target
	Motion new_motion;
	Ref_p_vec::iterator target_iter = move_to_closest_target(
		*cloned_planner,
		location,
		env->get_target_configurations(),
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
		env->get_target_configurations().erase(target_iter);
	} else {
		// Cut the motion
		new_motion.cut_motion(deadline - timer.time(), motion_output);
		// Update location to end of new motion
		if (!motion_output.empty()) {
			location = motion_output.back()->target();
		}
		is_last_motion_complete = false;
	}
	return true;
}

// Returns whether the player thinks that the game is over.
bool FastCopyingPlayer::is_game_over() {
	return !has_remaining_targets();
}

void FastCopyingPlayer::additional_targets_preprocessing(Ref_p_vec& additional_targets) {
	original_planner.preprocess_targets(additional_targets);
}

void FastCopyingPlayer::reject_last_move(Motion& motion_sequence) {
	// If the last (rejected) motion was to a target
	if (is_last_motion_complete) {
		// Return that target to the list of remaining targets
		env->get_target_configurations().push_back(last_target);
	}
}

// Initializes the underlying planner.
// Returns true iff the planner was initialized on this invocation.
bool FastCopyingPlayer::initialize() {
	if (planner_initialized) {
		return false;
	}

	original_planner.preprocess();
	// Allow additional preprocessing to source point and all initial targets
	sample_location_and_remaining_targets(original_planner);

	planner_initialized = true;
	return true;
}

void FastCopyingPlayer::improve_connectivity(Planner& planner, Reference_point& location, Ref_p_vec& targets) {
	planner.additional_preprocessing(location, targets);
}

void FastCopyingPlayer::sample_location_and_remaining_targets(Planner& planner) {
	// Allow additional preprocessing to source point and all initial targets
	Ref_p_vec source_and_targets = env->get_target_configurations();
	source_and_targets.push_back(location);
	original_planner.preprocess_targets(source_and_targets);
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
	return !env->get_target_configurations().empty();
}

// Are there remaining targets that are reachable?
bool FastCopyingPlayer::has_reachable_targets(Planner& planner) {
	return (!env->get_target_configurations().empty()
		&& planner.exist_reachable_target(location, env->get_target_configurations()));
}

// Are there remaining targets that are unreachable?
bool FastCopyingPlayer::has_unreachable_targets(Planner& planner) {
	return (!env->get_target_configurations().empty()
		&& !planner.exist_reachable_target(location, env->get_target_configurations()));
}
