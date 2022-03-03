#Martin Økter
#2021 mai

import time
import math
import machine #import PWM, ADC, Pin

led = machine.Pin(25, machine.Pin.OUT)

def testtest():
    for i in range(8):
        led.toggle()
        time.sleep(0.5)
testtest()

#Setup i2c conection
address = 0x68
address_ard = 0x08
sda = machine.Pin(0)
scl = machine.Pin(1)
bus = machine.I2C(0,scl=scl,sda=sda,freq=400000)

throttle = 0
arm = 0
wheel_one = 0
wheel_two = 0

#Set values
setAnglePitch = 0
setAngleRoll = 0
alpha = 0.98

#Set the initial time step
dt = 0.01

def ardu_map(val, in_min, in_max, out_min, out_max):
    #Returning float
    return max(min(out_max,(val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min), out_min)
    
def ard_read(addr):
    val = bus.readfrom(addr, 4)
    
    thr = (val[0:1])
    who = (val[1:2])
    arm = (val[2:3])
    wht = (val[3:4])
    
    arm = int(ardu_map(arm[0], 0, 255, 0, 10))
    
    return thr[0], arm, who[0], wht[0]
 
def angle_calc(acx, acy, acz):
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

def comp_pass(alpha, last_val, gyro_val, acc_val):
    return alpha * (last_val + gyro_val * dt) + (1-alpha)*acc_val

def PID(in_val, last_val, i_last, k_list):
    '''
    This function return the values needed
    for calcualting th PID tuning mekanism.
    -Økter
    '''
    p_val = k_list[0] * in_val
    i_mel = i_last + ((last_val + in_val)/2) * dt
    i_val = i_mel * k_list[1]
    d_val = ((in_val - last_val)/dt) * k_list[2]
    
    val = p_val + i_val + d_val
    return val, i_mel

class acsHandle:    
    def __init__(self):
        self.x_last = 0
        self.y_last = 0
        self.acs_list = [0x3B,0x3C,0x3D,0x3E,0x3F,0x40]
        self.gyro_list = [0x43,0x44,0x45,0x46,0x47,0x48]
        self.setup_list =  [0x6B,0x19,0x1A,0x1B,0x38]
        self.MPU_init()

    def MPU_init(self):
        bus.writeto_mem(address, self.setup_list[0], bytes([7]))
        bus.writeto_mem(address, self.setup_list[1], bytes([1]))
        bus.writeto_mem(address, self.setup_list[2], bytes([0]))
        bus.writeto_mem(address, self.setup_list[3], bytes([24]))
        bus.writeto_mem(address, self.setup_list[4], bytes([1]))
        
    def read_raw_data(self, addr, addr2):
        high = bus.readfrom_mem(address, addr,1)
        low = bus.readfrom_mem(address, addr2,1)
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

        return AcX, AcY, AcZ
    
    def gyro_read(self):
        gx = self.read_raw_data(self.gyro_list[0], self.gyro_list[1])
        gy = self.read_raw_data(self.gyro_list[2], self.gyro_list[3])
        gz = self.read_raw_data(self.gyro_list[4], self.gyro_list[5])

        gyrox = -1 * gx/131.0 * dt
        gyroy = -1 * gy/131.0 * dt
        gyroz = -1 * gz/131.0 * dt

        return gyrox, gyroy, gyroz
    
    def read(self):
        #try:
        acX, acY, acZ = self.acs_read()
        gx, gy, gz = self.gyro_read()
        #except:
            #rint('!!!!!!!!!!!!!!!!!!!')
            #break
        
        #Calculating angls from accs
        roll, pitch = angle_calc(acY, acX, acZ)
        
        #Compansating angle with gyro read
        angle_x = round(comp_pass(alpha, self.x_last, gx,pitch), 3)
        angle_y = round(comp_pass(alpha, self.y_last, gy,roll), 3)
        
        #Updating variables
        self.x_last = angle_x
        self.y_last = angle_y
        
        return angle_x, angle_y, gx, gy

class MotorHandle:
    def __init__(self, mot_list, freqs, range_list):
        self.low_freq = range_list[0]
        self.min_pwm = range_list[1]
        self.max_pwm = range_list[2]
        self.mot = []
        self.spin = []
        
        for i in range(len(mot_list)):
            self.mot.append(machine.PWM(machine.Pin(mot_list[i])))
        
        for item in self.mot:
            item.freq(freqs)
            self.spin.append(self.low_freq)
    
    def ardu_map(self, val, in_min, in_max):
        return int(max(min(self.max_pwm,(val - in_min) * (self.max_pwm - self.min_pwm) // (self.max_pwm - in_min) + self.min_pwm), self.min_pwm))
    
    def speed_calc(self, init, corPitch, corRoll):
        if len(self.mot) == 4:
            self.spin[0] = self.ardu_map(init + corPitch + corRoll,self.min_pwm,self.max_pwm)
            self.spin[1] = self.ardu_map(init - corPitch + corRoll,self.min_pwm,self.max_pwm)
            self.spin[2] = self.ardu_map(init + corPitch - corRoll,self.min_pwm,self.max_pwm)
            self.spin[3] = self.ardu_map(init - corPitch - corRoll,self.min_pwm,self.max_pwm)
    
    def speedy(self, throt, arm, corPitch, corRoll):
        init = self.ardu_map(throttle,0,250)
        
        if arm < 6:
            for item in self.mot:
                item.duty_u16(self.low_freq)
        else:
            #Should probably be moved out for calc when disarmed but for test it is placed here
            self.speed_calc(init, corPitch, corRoll)
            count = 0
            for item in self.mot:
                item.duty_u16(self.spin[count])
                count += 1

class LoopHandle:
    global dt
    
    def __init__(self,pid_list):
        self.pitch_i_last = 0
        self.roll_i_last = 0
        self.pitch_last_val = 0
        self.roll_last_val = 0
        self.p_list = pid_list[0:3]
        self.r_list = pid_list[3:6]
        
    def ang_calc(self,x,y,sX,sY):
        error = x - sX
        corrPi, i_mel = PID(error, self.pitch_last_val, self.pitch_i_last, self.p_list)
        self.pitch_last_val = corrPi
        
        if True: #i_val < 30 and i_val > 0: #Possible cutting of integrator
            self.pitch_i_last = i_mel
            
        error = y - sY
        corrRo, i_mel = PID(error, self.roll_last_val, self.roll_i_last, self.r_list)
        self.roll_last_val = corrRo
        
        if True: #i_val < 30 and i_val > 0: #Possible cutting of integrator
            self.roll_i_last = i_mel
            
        return corrPi, corrRo

Motor = MotorHandle([2,3,4,5],2000,[10000,19000,20000])

Loop1PID = [0.8,0.05,0,0.8,0.05,0]
Loop2PID = [0.8,0.05,0,0.8,0.05,0]

Loop1 = LoopHandle(Loop1PID)
Loop2 = LoopHandle(Loop2PID)

Acs1 = acsHandle()


cont = 0
tic = time.time()
count = 10;

while True:
    #Messuring values
    if count == 10:
        throttle, arm, wheel_one, wheel_two = ard_read(address_ard)
        Loop1.p_list[0] = ardu_map(wheel_one,0,250,0,10)
        Loop2.r_list[0] = ardu_map(wheel_two,0,250,0,10)
        count = 0
    count += 1
    
    #Updating variables
    angle_x, angle_y, gx, gy = Acs1.read()
    
    #Angle loop cascade (angle)
    corAngPitch, corAngRoll = Loop2.ang_calc(angle_x, angle_y, setAnglePitch, setAngleRoll)
     
    #Inner loop cascade (angleRate)  
    corRatePitch, corRateRoll = Loop1.ang_calc(corAngPitch, corAngRoll, gx, gy)
    
    #Throttling motros
    Motor.speedy(throttle,arm,corRatePitch,corRateRoll)
    
    #Time the loop
    if time.time() - tic == 0:
        cont += 1
    else:
        print(cont)
        dt = 1/cont
        cont = 0
        tic += 1
        
    
    time.sleep(0.1)
    #print("1: {} 2: {} 3: {} 4: {}".format(motor1speed,motor2speed,motor3speed,motor4speed))
    print("X angle: {} - Y angle {}".format(angle_x,angle_y))

    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
