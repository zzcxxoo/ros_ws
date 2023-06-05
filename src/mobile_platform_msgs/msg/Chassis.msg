std_msgs/Header header

uint8 driving_mode
uint8 VCU_EMERGENCY_STOP = 0
uint8 HUMAN_MANUAL = 1
uint8 VCU_MANUAL = 2
uint8 IPC_EMERGENCY = 3
uint8 IPC_MANUAL = 4
uint8 IPC_AUTO = 5
uint8 IPC_CLIENT = 6

mobile_platform_msgs/DriveCommand drive_command
mobile_platform_msgs/VCU vcu
mobile_platform_msgs/Motor mcu
mobile_platform_msgs/EPS eps
mobile_platform_msgs/WheelEncoder encoder
mobile_platform_msgs/BMS bms
mobile_platform_msgs/Surround surround
mobile_platform_msgs/Light light
mobile_platform_msgs/SweeperAuxilaries sweeper_aux
float64 odometer_m

mobile_platform_msgs/ErrorCode[] error_codes







