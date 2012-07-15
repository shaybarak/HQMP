#ifndef CLIENT_UTILS_H
#define CLIENT_UTILS_H

#include <string>
#include "Utils\Communication_utils\Socket.h"
#include "Utils\Communication_utils\Socket_typedefs.h"

Time_frame_status get_time_frame_status(Socket_client& socket_client);
Scene_status get_scene_status(Socket_client& socket_client);
bool request_to_write(Socket_client& socket_client, double time, std::string& path_filename);
void terminate_connection(Socket_client& socket_client);
#endif //CLIENT_UTILS_H