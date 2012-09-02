#pragma once

#include "Player.h"
#include "Path_planning\Motion_sequence.h"

/**
 * Player that buffers motion ahead during the planning phase.
 */
class FastCopyingPlayer : protected Player {
	typedef Planner::Extended_polygon	Extended_polygon;
	typedef Planner::Reference_point	Reference_point;
	typedef Env::Reference_point_vec	Ref_p_vec;

public:
	FastCopyingPlayer(Env* env, Configuration* config);
	~FastCopyingPlayer();
	virtual bool plan(double deadline);
	virtual bool move(double deadline, Motion& motion_output);
	virtual bool is_game_over();
	virtual void additional_targets_preprocessing(Ref_p_vec& additional_targets);
	virtual void reject_last_move(Motion& motion_sequence);

private:
	// Initializes the underlying planner if not initialized already
	bool initialize();

	// Samples the robot's current configuration
	void sample_current_location();

	// Improves a planner's connectivity from the current location to target configurations
	void improve_connectivity(Planner& planner,
		Reference_point& location,
		Ref_p_vec& targets);

	// Clones the original planner, taking into account the other robot's current location
	void clone_planner();

	// Move from source to the closest target.
	// Appends new motion sequence to motion.
	Ref_p_vec::iterator move_to_closest_target(
		Planner& planner,
		Reference_point& source,
		Ref_p_vec& targets,
		Motion& motion);

	// Useful predicates
	bool has_remaining_targets();
	bool has_reachable_targets(Planner& planner);
	bool has_unreachable_targets(Planner& planner);
	Ref_p_vec& remaining_targets();

	// Original planner without an obstacle
	Planner original_planner;

	// Cloned planner with an obstacle
	Planner* cloned_planner;
	
	// Our current location
	Reference_point location;

	// The dynamic obstacle that the planner was cloned for
	Reference_point cached_dynamic_obstacle;
	
	// Caches whether the planner was initialized
	bool planner_initialized;

	// Workaround for rejecting the last move
	// Whether the last motion was returned completely
	bool is_last_motion_complete;
	// The last target that was reached
	Reference_point last_target;
};
