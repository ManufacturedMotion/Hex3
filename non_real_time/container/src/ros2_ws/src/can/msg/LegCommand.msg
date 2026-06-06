# LegCommand message for hexapod leg commands
# Command type constants
uint8 LINEAR=16
uint8 RAPID=20
uint8 SINGLE_AXIS=19

# Which command this message contains
uint8 command_type

# For SINGLE_AXIS: axis index (0..255). For others, ignored.
uint8 axis

# Position/velocity values. Use fields appropriate to the command_type.
float32 x
float32 y
float32 z
float32 speed

# For SINGLE_AXIS commands, the target position on the given axis.
float32 position
