import math
import numpy as np

def pwm2thrust_dji_set(pwm, voltage):
    pwm_mean = 1525
    pwm_std = 231.1462
    voltage_mean = 11.7894
    voltage_std = 0.33
    p00 = 2.939
    p10 = 2.422
    p01 = 0.1464
    p20 = 0.3989
    p11 = 0.1119

    x = (pwm - pwm_mean) / pwm_std
    y = (voltage - voltage_mean) / voltage_std

    thrust = p00 + p10 * x + p01 * y + p20 * (x ** 2) + p11 * x * y

    return thrust

def thrust2pwm_dji_set(thrust, voltage):
    pwm_mean = 1525
    pwm_std = 231.1462
    voltage_mean = 11.7894
    voltage_std = 0.33
    p00 = 2.939
    p10 = 2.422
    p01 = 0.1464
    p20 = 0.3989
    p11 = 0.1119

    y = (voltage - voltage_mean) / voltage_std

    a = p20
    b = p11 * y + p10
    c = p00 + p01 * y - thrust
    delta = b * b - 4.0 * a * c
    
    if delta < 0.0:
         return 1000.0
    else:
        x = (-b + math.sqrt(delta)) / 2.0 / a
        pwm = x * pwm_std + pwm_mean
        pwm = np.clip(pwm, 1100.0, 1900.0)
        return pwm