#include "stdafx.h"
#include "NaivePlayer.h"
 
NaivePlayer::NaivePlayer(Env* env, Configuration* config) :
	Player(env, config),
	planner(env->get_workspace(),
	env->get_robot_a()),
	q_s(env->get_source_configuration_a()),
	planned(false) {
}
 
void NaivePlayer::plan(double deadline) {
        // Don't allow more than one planning turn
        if (!planned) {
                // Assume we have enough time to preprocess
                planner.preprocess();
                planned = true;
        }
        plan_future_motion_seq();
}
 
 
void NaivePlayer::plan_future_motion_seq(){
 
        Planner::Reference_point last_point_in_path;
        Ref_p_vec::iterator iter_all_targets;
        Ref_p_vec::iterator iter_to_closest;
        Motion total_motion;
        bool path_found;
		int target_index;
       
        if (env->get_target_configurations().empty()){
                //No more motion to plot
                return;
        }
		
		if (remaining_motion.empty()){
			last_point_in_path = q_s;
		}
       // Find motion to closest target
        path_found = planner.query_closest_point(
                        last_point_in_path,
                        env->get_target_configurations(),
                        target_index,
                        total_motion);
	
 
        if (path_found){
			iter_to_closest = env->get_target_configurations().begin() + target_index;
			last_point_in_path = *iter_to_closest;
            //Erase next target
            env->get_target_configurations().erase(iter_to_closest);
            remaining_motion.add_motion_sequence(total_motion);	   
        }      
}      
 
 
 
void NaivePlayer::move(double deadline, Motion& motion_sequence) {
 
        CGAL::Timer timer;
        timer.start();
        bool path_found = true;

		if (!remaining_motion.empty()) {
			// Continue a previous motion
			remaining_motion.cut(deadline - timer.time(), configuration.get_translational_speed(), configuration.get_rotational_speed(), motion_sequence);
			std::cout << "Cut to " << motion_sequence.get_sequence().size() << " and " << remaining_motion.get_sequence().size() << endl;
			if (!motion_sequence.empty()){
					q_s = motion_sequence.get_sequence().back()->target();
			}
			return;
		}
 
        if (env->get_target_configurations().empty()) {
                // No more motion to plot
                return;
        }
 
		plan(deadline);

       	if (!remaining_motion.empty()) {
			// Cut it according to the deadline
			remaining_motion.cut(deadline - timer.time(), configuration.get_translational_speed(), configuration.get_rotational_speed(), motion_sequence);
			std::cout << "Cut to " << motion_sequence.get_sequence().size() << " and " << remaining_motion.get_sequence().size() << endl;
			if (!motion_sequence.empty()){
					q_s = motion_sequence.get_sequence().back()->target();
			}
			return;
		}
 

         // TODO maybe the motion is empty because the path is (temporarily) blocked, fix this in a later iteration
        return;           
}

// Returns whether the player thinks that the game is over.
bool NaivePlayer::is_game_over() {
	return (
		// No additional targets
		env->get_target_configurations().empty() &&
		// No planned pending motion
		remaining_motion.empty());
}
