#include "stdafx.h"
#include "NaivePlayer.h"

NaivePlayer::NaivePlayer(Env* env, Configuration* config) : Player(env, config), planner(env->get_workspace(), env->get_robot_a()), q_s(env->get_source_configuration_a()) {
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

	CGAL::Timer timer;
	timer.start();
	
	Ref_p_vec::iterator iter_all_targets;
	Ref_p_vec::iterator iter_to_closest;
	Motion total_motion;
	
	// Assume we have enough time to perform the entire motion
	if (!env->get_target_configurations().empty()){
	
		// Perform query
		bool path_found = planner.query_closest_point(
			q_s, 
			env->get_target_configurations(),
			iter_to_closest, 
			total_motion);

		q_s = *iter_to_closest;

		//delete reached target from target_configurations vector
		iter_all_targets = env->get_target_configurations().begin();
		while (*iter_all_targets != *iter_to_closest){
			iter_all_targets++;
		}
		env->get_target_configurations().erase(iter_all_targets);


		if (path_found) {
			std::cout << "Found path with " << total_motion.get_sequence().size() << " steps requiring " << total_motion.motion_time(configuration.get_translational_speed(), configuration.get_rotational_speed()) << " seconds" << endl;
			// Cut it according to the deadline
			total_motion.cut(deadline - timer.time(), configuration.get_translational_speed(), configuration.get_rotational_speed(), motion_sequence);
			std::cout << "Cut to " << motion_sequence.get_sequence().size() << " and " << total_motion.get_sequence().size() << endl;
		}
		// If we did not generate a motion, assume that the game is over
		// TODO maybe the motion is empty because the path is (temporarily) blocked, fix this in a later iteration
		return motion_sequence.empty();
	}else{
		return true;
	}
	
}
