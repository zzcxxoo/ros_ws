std_msgs/Header header

float64 front_wheel_angle         # clockwise:negative, anticlockwise: positive
float64 steering_angle            # clockwise:negative, anticlockwise: positive
float64 steering_angle_speed
float64 torque

uint8 working_mode
uint8 TORQUE_CONTROL = 0
uint8 SPEED_CONTROL = 1
uint8 ANGLE_CONTROL = 2

bool is_angle_holding
uint8 is_middle_point_calibrated

uint8 eps_fault
