# Referee system status forwarded from the Type-C.
# Sent by the Type-C at ~5 Hz interleaved with POSE_MSG (REF_SYS_MSG, id=3).
std_msgs/Header header

uint8  game_stage               # referee game stage enum value
uint16 stage_time_remaining     # seconds remaining in current stage
uint16 robot_hp                 # current robot HP
uint8  robot_id                 # robot ID normalised to red-team numbering (hero=1)
float32 delta_angle_got_hit_in  # angle from current heading at which last hit arrived (radians)

# Booleans unpacked from the single booleans byte.
# Bit layout on the wire (MSB→LSB): blue|healing|reload|center|teamCenter|oppCenter|chassis|gimbal
bool is_on_blue_team
bool is_healing
bool is_in_reload_zone          # restoration or exchange zone RFID active
bool is_in_center_zone          # central buff RFID active
bool team_occupies_center
bool opponent_occupies_center
bool chassis_has_power
bool gimbal_has_power
