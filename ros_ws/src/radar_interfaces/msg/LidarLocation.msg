# LidarLocation — mirrors radar_bridge zmq_data_format.hpp
# PUB direction: /lidar/pose → radar_bridge → ZMQ → radar-egui

uint16 cmd_id

# opponent
uint16 opponent_hero_x
uint16 opponent_hero_y
uint16 opponent_engineer_x
uint16 opponent_engineer_y
uint16 opponent_infantry_3_x
uint16 opponent_infantry_3_y
uint16 opponent_infantry_4_x
uint16 opponent_infantry_4_y
uint16 opponent_aerial_x
uint16 opponent_aerial_y
uint16 opponent_sentry_x
uint16 opponent_sentry_y

# ally
uint16 ally_hero_x
uint16 ally_hero_y
uint16 ally_engineer_x
uint16 ally_engineer_y
uint16 ally_infantry_3_x
uint16 ally_infantry_3_y
uint16 ally_infantry_4_x
uint16 ally_infantry_4_y
uint16 ally_aerial_x
uint16 ally_aerial_y
uint16 ally_sentry_x
uint16 ally_sentry_y
