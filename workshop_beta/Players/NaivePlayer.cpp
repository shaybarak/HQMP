#include "stdafx.h"
#include "NaivePlayer.h"

NaivePlayer::NaivePlayer(Env* env, Configuration* config) : Player(env, config), planner(env->get_workspace(), env->get_robot_a()) {
	planned = false;
	moved = false;
}

void NaivePlayer::plan(double deadline) {
	// Don't allow more than one planning turn
	if (!planned) {
		// Assume we have enough time to preprocess
		planner.preprocess();
		planned = true;
	}
}

bool NaivePlayer::move(double deadline, Motion& motion_sequence) {
	if (moved) {
		return true;
	}

	CGAL::Timer timer;
	timer.start();

	// Don't allow more than one movement turn
	// Assume we have enough time to perform the entire motion
	// Get source the destination
	Planner::Reference_point q_s (env->get_source_configuration_a());
	Planner::Reference_point q_t (env->get_target_configurations().front());

	// Perform query
	Ref_p_vec::iterator iter_to_closest;
	Motion total_motion;
	bool path_found = planner.query_closest_point(
		q_s, 
		env->get_target_configurations(),
		iter_to_closest, 
		total_motion);
	
	if (path_found) {
		std::cout << "Found path with " << total_motion.get_sequence().size() << " steps requiring " << total_motion.motion_time(configuration.get_translational_speed(), configuration.get_rotational_speed()) << " seconds";
		// Cut it according to the deadline
		total_motion.cut(deadline - timer.time(), configuration.get_translational_speed(), configuration.get_rotational_speed(), motion_sequence);
		std::cout << "Cut to " << motion_sequence.get_sequence().size() << " and " << remaining.get_sequence().size();
	}

	moved = true;
	
	// If we did not generate a motion, assume that the game is over
	// TODO maybe the motion is empty because the path is (temporarily) blocked, fix this in a later iteration
	return motion_sequence.empty();
}
