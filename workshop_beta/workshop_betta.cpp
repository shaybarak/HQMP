#include "stdafx.h"
#include "Programs\Path_planner.h"
#include "Programs\Server.h"
#include "Programs\Client_stubs.h"

int main(int argc, char* argv[])
{
  //single_robot_planner_example(argc, argv);
  server_main(argc, argv);
  //client_stubs_main(argc, argv);
    
	return 0;
}

