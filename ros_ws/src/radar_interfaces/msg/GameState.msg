# TransmitGameState — mirrors radar_bridge zmq_data_format.hpp
# SUB direction: radar-egui → ZMQ → radar_bridge → this ROS topic

uint16 cmd_id
uint8 game_type
uint8 game_progress
uint16 stage_remain_time
uint64 sync_timestamp
