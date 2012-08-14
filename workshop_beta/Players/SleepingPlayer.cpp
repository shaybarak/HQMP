#include "stdafx.h"
#include "SleepingPlayer.h"

void SleepingPlayer::plan(double deadline) {
	// Do nothing, just sleep
	boost::posix_time::seconds sleep_time(deadline);
	boost::this_thread::sleep(sleep_time);
}

bool SleepingPlayer::move(double deadline, Motion& motion_sequence) {
	// Do nothing, just sleep
	boost::posix_time::seconds sleep_time(deadline);
	boost::this_thread::sleep(sleep_time);
	return false;
}
