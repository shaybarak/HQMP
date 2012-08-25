#include "stdafx.h"
#include "SleepingPlayer.h"

bool SleepingPlayer::plan(double deadline) {
	// Do nothing, just sleep
	boost::posix_time::seconds sleep_time(deadline);
	boost::this_thread::sleep(sleep_time);
	return false;
}

bool SleepingPlayer::move(double deadline, Motion& motion_sequence) {
	// Do nothing, just sleep
	boost::posix_time::seconds sleep_time(deadline);
	boost::this_thread::sleep(sleep_time);
	return false;
}

bool SleepingPlayer::is_game_over() {
	return false;
}

void SleepingPlayer::additional_targets_preprocessing(Ref_p_vec& additional_targets) {}
