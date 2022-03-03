import machine
import time

address = 0x68
ACCEL_XOUT = 0x3B
sda = machine.Pin(0)
scl = machine.Pin(1)
bus = machine.I2C(0,scl=scl,sda=sda,freq=400000)

PWR_MGMT = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38

def MPU_init():
    bus.writeto_mem(address, SMPLRT_DIV, bytes([7]))
    bus.writeto_mem(address, PWR_MGMT, bytes([1]))
    bus.writeto_mem(address, CONFIG, bytes([0]))
    bus.writeto_mem(address, GYRO_CONFIG, bytes([24]))
    bus.writeto_mem(address, INT_ENABLE, bytes([1]))

MPU_init()

time.sleep(2)
# devices = bus.scan()
# 
# print("Number of devises: ",len(devices))
# 
# for device in devices:
#    print("Decimal addres: ",device," | Hexa address: ",hex(device))


while True:
    #temp_bit = bus.readfrom(address, 4)
    temp_high = bus.readfrom_mem(address, 0x3B,1)
    temp_low = bus.readfrom_mem(address, 0x3C,1)
    temp = (temp_high[0] << 8) | temp_low[0]
#     temp_thr = (temp_bit[0:1])
#     temp_arm = (temp_bit[1:2])
#     temp_who = (temp_bit[2:3])
#     temp_wht = (temp_bit[3:4])
    #temp = (temp_high[0] << 8) | temp_low[0]
    #temp = temp_low[0] + t
    
    #ẗemp_high[0] * 0.001
    print(temp)
    #print('Throttle {} ARM {} Wheel-one {} Wheel-two {}'.format(temp_thr[0],temp_arm[0],temp_who[0],temp_wht[0]))
    time.sleep(0.1)

