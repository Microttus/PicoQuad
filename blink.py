from machine import PWM, ADC, Pin
from time import sleep

motor = PWM(Pin(2))
motor.freq(2000)
adc = ADC(Pin(26))

def ardu_map(val, in_min, in_max, out_min, out_max):
    return max(min(out_max,(val - in_min) * (out_max - out_min) // (in_max - in_min) + out_min), out_min)

while True:
    #read_val = adc.read_u16()
    #mot_val = ardu_map(read_val, 400, 35000, 0, 65000)
    #print(mot_val)
    w_val = int(input("Fart: "))
    #print(mot_val)
    motor.duty_u16(w_val)    
    sleep(0.1)
