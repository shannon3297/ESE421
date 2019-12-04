import numpy as np
import math
import Arduino_Communicator
import Kalman

# create Arduino communicator and Kalman objects
arduinoCom = ArduinoCommunicator(0x8)
kalman = Kalman()
while(true):
    r_imu = get_arduino_data_from_register(arduinoCom, 1)
    velocity = get_arduino_data_from_register(arduinoCom, 2)
    dt_kalman = get_arduino_data_from_register(arduinoCom, 3)
    u = np.array([r_imu, velocity])
    x_k = kalman.predictionNoCamera(u, dt_kalman)
    coneX = 50
    coneY = 50
    pwm_angle = kalman.sendToArduino(x_k, coneX, coneY)
    arduinoCom.write_float_to_register(pwm_angle[0], 4) # register 4 stores PWM
    arduinoCom.write_float_to_register(pwm_angle[1], 5) # register 5 stores servoAngleDeg
    arduinoCom.trigger_method("SENT")
