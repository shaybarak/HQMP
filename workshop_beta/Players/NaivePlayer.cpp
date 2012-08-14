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

void NaivePlayer::move(double deadline, Motion& motion_sequence) {
	// TODO use a clock internally to also

	// Don't allow more than one movement turn
	if (!moved) {
		// Assume we have enough time to perform the entire motion
		// Get source the destination
		Planner::Reference_point q_s (env->get_source_configuration_a());
		Planner::Reference_point q_t (env->get_target_configurations().front());

		// Perform query
		Motion remaining;
		Ref_p_vec::iterator iter_to_closest;

		//This is the original one query version. can uncomment out for debug. 
		//TODO: remove.
		//bool found_path = planner.query(q_s, q_t, motion_sequence);
		bool path_found = planner.query_closest_point(
			q_s, 
			env->get_target_configurations(),
			iter_to_closest, 
			motion_sequence);

		if (path_found) {
			// Path found!
			std::cout << "Found path with " << remaining.get_sequence().size() << " steps requiring " << remaining.motion_time(configuration.get_translational_speed(), configuration.get_rotational_speed()) << " seconds";
			// Cut it according to the deadline
			remaining.cut(deadline, configuration.get_translational_speed(), configuration.get_rotational_speed(), motion_sequence);
			std::cout << "Cut to " << motion_sequence.get_sequence().size() << " and " << remaining.get_sequence().size();
		}

		moved = true;
	}
}
