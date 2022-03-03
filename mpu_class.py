import math
import machine
import time

class acsHandle:
    '''
    Class for reading the MPU6050 on an Raspberry Pi pico
    '''
    def __init__(self, pin_list):
        self.bus = machine.I2C(0,scl=machine.Pin(pin_list[1]),sda=machine.Pin(pin_list[0]),freq=400000)
        self.address = 0x68
        self.x_last = 0
        self.y_last = 0
        self.alpha = 0.99
        self.acs_list = [59,60,61,62,63,64]
        self.gyro_list = [67,68,69,70,71,72]
        
        self.MPU_init()

    def MPU_init(self):
        self.bus.writeto_mem(self.address, 25, bytes([7]))
        self.bus.writeto_mem(self.address, 107, bytes([1]))
        self.bus.writeto_mem(self.address, 26, bytes([0]))
        self.bus.writeto_mem(self.address, 27, bytes([24]))
        self.bus.writeto_mem(self.address, 56, bytes([1]))
        
    def read_raw_data(self, addr, addr2):
        high = self.bus.readfrom_mem(0x68, 0x3B,1)
        low = self.bus.readfrom_mem(0x68, 0x3C,1)
        value = ((high[0] << 8) | low[0])
        
        if (value > 32768):
            value = value - 65536
        return value   
    
    def acs_read(self):
        acX = self.read_raw_data(self.acs_list[0], self.acs_list[1])
        acY = self.read_raw_data(self.acs_list[2], self.acs_list[3])
        acZ = self.read_raw_data(self.acs_list[4], self.acs_list[5])
        
        AcX = acX/16384.0
        AcY = acY/16384.0
        AcZ = acZ/16384.0
        print(AcX)
        return AcX, AcY, AcZ
    
    def gyro_read(self):
        gx = self.read_raw_data(self.gyro_list[0], self.gyro_list[1])
        gy = self.read_raw_data(self.gyro_list[2], self.gyro_list[3])
        gz = self.read_raw_data(self.gyro_list[4], self.gyro_list[5])

        gyrox = -1 * gx/131.0 * dt
        gyroy = -1 * gy/131.0 * dt
        gyroz = -1 * gz/131.0 * dt

        return gyrox, gyroy, gyroz
    
    def angle_calc(self,acx, acy, acz):
        if acx == 0:
            acx = 0.001
        elif acy == 0:
            acy = 0.001
        elif acz == 0:
            acz = 0.001
        roll = math.atan2(acy,acz)
        pitch = -1 * math.atan2(acx ,(math.sqrt(acy**2+acz**2)))
        
        pitch = pitch * (180/math.pi)
        roll = roll * (180/math.pi)
        return roll, pitch
    
    def comp_pass(self, last_val, gyro_val, acc_val):
        return self.alpha * (last_val + gyro_val * dt) + (1-self.alpha)*acc_val
    
    def read(self):
        #try:
        acX, acY, acZ = self.acs_read()
        gx, gy, gz = self.gyro_read()
        #except:
            #print('!!!!!!!!!!!!!!!!!!!')
            #break
       
        #Calculating angls from accs
        roll, pitch = self.angle_calc(acX, acY, acZ)
        
        #print('Roll: {}, Pitch: {}'.format(roll,pitch))
        #Compansating angle with gyro read
        angle_x = round(self.comp_pass(self.x_last, gx,pitch), 3)
        angle_y = round(self.comp_pass(self.y_last, gy,roll), 3)
        
        #Updating variables
        self.x_last = angle_x
        self.y_last = angle_y
        
        return angle_x, angle_y, gx, gy
    
i2c_pins = [0,1]
Acs1 = acsHandle(i2c_pins)

dt = 1

if __name__ == "__main__":
    while True:
        angle_x, angle_y, gx, gy = Acs1.read()
        #print("Angle X: {}, Angle Y: {}".format(angle_x,angle_y))
        time.sleep(1)
        







