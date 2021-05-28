Header header

int8 MOTOR_ID_FRONT_RIGHT = 0
int8 MOTOR_ID_FRONT_LEFT = 1
int8 MOTOR_ID_REAR_RIGHT = 2
int8 MOTOR_ID_REAR_LEFT = 3

int8 LIGHT_ID_FRONT = 0
int8 LIGHT_ID_REAR = 1

# motion state
MotionStateMsg motion_state

# actuator state
ActuatorStateMsg[4] actuator_state

# light state
bool light_control_enabled
LightControlType light_state

#system state
SystemStateMsg system_state