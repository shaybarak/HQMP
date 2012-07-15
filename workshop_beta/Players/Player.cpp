#include "stdafx.h"
#include "Player.h"

Player::Player(Env* env, Configuration* config) {
	this->env = env;
	this->config = config;
}

void Player::set_dynamic_obstacle_config(Ref_p& dynamic_obstacle) {
	this->dynamic_obstacle = dynamic_obstacle;
}

void Player::update_target_configs(Ref_p_vec* target_configs) {
	this->target_configs = target_configs;
}

void Player::log(char* label, char* message) {
	std::cout << label << ": " << message << std::endl;
}
