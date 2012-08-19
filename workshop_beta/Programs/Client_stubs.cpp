#include "stdafx.h"
#include "Programs\client_stubs.h"

#include "Utils\Communication_utils\socket.h"
#include "Utils\Communication_utils\client_utils.h"

#include "Mms_example.h"
#include "Path_planning\Motion_sequence.h"

#include <boost/thread/thread.hpp>

#include "..\Players\Player.h"
#include "..\Players\SleepingPlayer.h"
#include "..\Players\NaivePlayer.h"
#include "..\Players\MyPlayer.h"
#include "..\Utils\logging.h"

//global typedefs used in this file
typedef Environment<>                     Env;
typedef Motion_sequence<Planner::K>       Motion;
typedef Env::Reference_point              Ref_p;

//globals for the sake of the example - bad programing!
Socket_client*            socket_client_ptr = NULL;
Player*					  player;
bool                      finished_game = false;

void plan(double remaining_time)
{
	//here you should implement your planner
	//instead we just sleep for the remaining time

	CGAL::Timer timer;
	timer.start();


	if (finished_game)  //update this flag when you finished all queries
		return;
	
	while((timer.time() < remaining_time) && !finished_game) {
		player->plan(remaining_time - timer.time());
	}

	return;
}

void construct_motion(double remaining_time, Motion& motion_sequence)
{
	//construct a motion, 
	//make sure that you have enough time to complete the motion 
	//or else no write approval will granted

	player->move(remaining_time, motion_sequence);
	finished_game = player->is_game_over();

	return;
}

// Returns whether this method should be called again this turn
// (if the remaining time permits)
bool move(double remaining_time)
{
	if (finished_game) {
		return false;
	}

	Motion motion_sequence;
	construct_motion(remaining_time, motion_sequence);

	double motion_length(motion_sequence.motion_time());
	if (motion_length == 0) //no motion
		return false;
	std::string path_filename;

	TIMED_TRACE_ACTION("move", "requesting to write move");
	if (request_to_write(*socket_client_ptr, motion_length, path_filename))
	{
		TIMED_TRACE_ACTION("move", "request granted");
		//request granted
		ofstream out(path_filename.c_str(), std::ios::app); //it is essential to append
		motion_sequence.write(out);    
	}
	else
	{
		TIMED_TRACE_ACTION("move", "request denied");
		// TODO handle movement failures correctly
	}

	return true;
}

void static_planner(double remaining_time)
{
	CGAL::Timer timer;
	timer.start();
	plan(remaining_time);
	cout << "Planned " << timer.time() << ", sleeping for the remaining " << remaining_time - timer.time() << endl;
	boost::posix_time::seconds sleep_time(remaining_time - timer.time());
	boost::this_thread::sleep(sleep_time);
	return;
}

void moveable_planner(double remaining_time)
{
	CGAL::Timer timer;
	timer.start();
	TIMED_TRACE_ENTER("moveable_planner");
	while((timer.time() < remaining_time) && !finished_game) {
		// Continue moving
		if (!move(remaining_time - timer.time())) {
			TIMED_TRACE_ACTION("moveable_planner", "could not find next move, spending rest of turn planning");
			// If there is no extra movement to do this turn, spend the remaining time planning
			static_planner(remaining_time - timer.time());
		}
	}
	TIMED_TRACE_EXIT("moveable_planner");
	return;
}

void client_stubs_main(int argc, char* argv[])
{
	////////////////////////////////////////////////////////////
	Env env(argc,argv);
	Input_reader input_reader;

	////////////////////////////////////////////////////////////
	// Initialize connection to server
	socket_client_ptr = new Socket_client(configuration.get_host_name(), configuration.get_host_port());

	////////////////////////////////////////////////////////////
	// Begin game
	player = (Player*)new MyPlayer(&env, &configuration);
	bool read_additional_configurations = true;
	while (is_game_over(*socket_client_ptr) == false) {    
		// Before doing any planning, see if something changed
		TIMED_TRACE_ACTION("client_main", "getting scene status");
		Scene_status scene_status = get_scene_status(*socket_client_ptr); 
		player->set_dynamic_obstacle_config(
			input_reader.read_reference_point<Rational_kernel>(scene_status.quasi_dynamic_obs_location_filename));
		if (scene_status.updated_target_configurations && (read_additional_configurations))
		{
			input_reader.read_reference_points<Rational_kernel>(
				scene_status.updated_target_configurations_filename,
				std::back_inserter(env.get_target_configurations()));
			read_additional_configurations = false;
			finished_game = false;
		}   

		// Now play
		TIMED_TRACE_ACTION("client_main", "playing turn");
		Time_frame_status time_frame_status = get_time_frame_status(*socket_client_ptr); 
		if (time_frame_status.is_moveable) {
			moveable_planner(time_frame_status.remaining_time);
		} else {
			static_planner(time_frame_status.remaining_time);
		}
		TIMED_TRACE_ACTION("client_main", "turn complete");
		cout << endl;
	}

	cout << "FINISHED..." << endl;

	terminate_connection(*socket_client_ptr);
	delete (socket_client_ptr);

	return;
}
