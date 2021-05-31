# All the set points for movement control
# The topic name for this is "mov_set_points"

# Set points for constant movement in thrust % [-100,100]
# To get movement in the opposite direction, just set the thrust % value to negative
float32 set_forward
float32 set_right

# Set points for desired position state
float32 set_x  # Surge set point
float32 set_y  # Sway set point
float32 set_z  # Depth/heave set point
float32 set_yaw
float32 set_pitch
float32 set_roll
