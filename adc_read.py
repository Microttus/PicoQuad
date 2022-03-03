from machine import PWM, ADC, Pin
from time import sleep

adc = ADC(Pin(26))
pwmin = Pin(26)
#p2 = Pin(26, Pin.IN, Pin.PULL_UP)

def ardu_map(val, in_min, in_max, out_min, out_max):
    return max(min(out_max,(val - in_min) * (out_max - out_min) // (in_max - in_min) + out_min), out_min)

while True:
    read_val = adc.read_u16()
    print(read_val)
    #print(red)
    sleep(0.1)
