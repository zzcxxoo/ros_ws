std_msgs/Header header

# Lights
uint8 top_light
uint8 front_light
uint8 low_beam
uint8 turn_left_light
uint8 turn_right_light
# Sound
uint8 horn_short
uint8 horn
# others effective only if auto clean OFF
uint8 auto_clean
uint8 side_brush
uint8 side_brush_pusher
uint8 main_brush
uint8 main_brush_pusher
uint8 fan
uint8 water_pump
uint8 water_pump_pwm
###
uint8 dust_shaker
uint8 rubber_pusher
uint8 rubber_pusher_extend_time

uint8 OFF = 0
uint8 ON = 1
uint8 FLASH = 2
uint8 RED = 3
uint8 GREEN = 4
uint8 RED_FLASH = 5
uint8 GREEN_FLASH = 6