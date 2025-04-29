from ultrasonic import Ultrasonic
from motor import Ordinary_Car
from servo import Servo
from infrared import Infrared
from adc import ADC
import time
import math

class Car:
    def __init__(self):
        self.servo = None
        self.sonic = None
        self.motor = None
        self.infrared = None
        self.adc = None
        self.car_record_time = time.time()
        self.car_sonic_servo_angle = 0
        self.car_sonic_servo_dir = 1
        self.car_sonic_distance = [30, 30, 30]
        self.time_compensate = 3 #Depend on your own car,If you want to get the best out of the rotation mode, change the value by experimenting.
        self.start()

    def start(self):
        if self.servo is None:
            self.servo = Servo()
        if self.sonic is None:
            self.sonic = Ultrasonic()
        if self.motor is None:
            self.motor = Ordinary_Car()
        if self.infrared is None:
            self.infrared = Infrared()
        if self.adc is None:
            self.adc = ADC()

    def close(self):
        self.motor.set_motor_model(0,0,0,0)
        self.sonic.close()
        self.motor.close()
        self.infrared.close()
        self.adc.close_i2c()
        self.servo = None
        self.sonic = None
        self.motor = None
        self.infrared = None
        self.adc = None

    def run_motor_ultrasonic(self, distance):
        if (distance[0] < 30 and distance[1] < 30 and distance[2] <30) or distance[1] < 30 :
            self.motor.set_motor_model(-1450,-1450,-1450,-1450)
            time.sleep(0.1)
            if distance[0] < distance[2]:
                self.motor.set_motor_model(1450,1450,-1450,-1450)
            else :
                self.motor.set_motor_model(-1450,-1450,1450,1450)
        elif distance[0] < 30 and distance[1] < 30:
            self.motor.set_motor_model(1500,1500,-1500,-1500)
        elif distance[2] < 30 and distance[1] < 30:
            self.motor.set_motor_model(-1500,-1500,1500,1500)
        elif distance[0] < 20 :
            self.motor.set_motor_model(2000,2000,-500,-500)
            if distance[0] < 10 :
                self.motor.set_motor_model(1500,1500,-1000,-1000)
        elif distance[2] < 20 :
            self.motor.set_motor_model(-500,-500,2000,2000)
            if distance[2] < 10 :
                self.motor.set_motor_model(-1500,-1500,1500,1500)
        else :
            self.motor.set_motor_model(600,600,600,600)

    def mode_ultrasonic(self):
        if (time.time() - self.car_record_time) > 0.2:
            self.car_record_time = time.time()
            self.servo.set_servo_pwm('0', self.car_sonic_servo_angle)
            if self.car_sonic_servo_angle == 30:
                self.car_sonic_distance[0] = self.sonic.get_distance()
            elif self.car_sonic_servo_angle == 90:
                self.car_sonic_distance[1] = self.sonic.get_distance()
            elif self.car_sonic_servo_angle == 150:
                self.car_sonic_distance[2] = self.sonic.get_distance()

            print("L:{}, M:{}, R:{}".format(self.car_sonic_distance[0], self.car_sonic_distance[1], self.car_sonic_distance[2]))
            self.run_motor_ultrasonic(self.car_sonic_distance)

            if self.car_sonic_servo_angle <= 0:
                self.car_sonic_servo_dir = 1
            elif self.car_sonic_servo_angle >= 120:
                self.car_sonic_servo_dir = 0
            if self.car_sonic_servo_dir == 1:
                self.car_sonic_servo_angle += 60
            elif self.car_sonic_servo_dir == 0:
                self.car_sonic_servo_angle -= 60

    def mode_infrared(self):
        if (time.time() - self.car_record_time) > 0.2:
            self.car_record_time = time.time()
            infrared_value = self.infrared.read_all_infrared()
            print("infrared_value: " + str(infrared_value))

            left_infrared = self.infrared.read_one_infrared(1) << 2
            right_infrared = self.infrared.read_one_infrared(3)
            center_infrared = self.infrared.read_one_infrared(2) << 1

            print("left_infrared: " + str(left_infrared), "right_infrared: " + str(right_infrared), "center_infrared: " + str(center_infrared))

            if center_infrared == 2:
                # Move forward in episodes
                self.motor.set_motor_model(800,800,800,800)  # Move forward
                time.sleep(0.2)  # Move for a short time
                self.motor.set_motor_model(0,0,0,0)  # Stop
                time.sleep(0.1)  # Pause to check sensors

            elif left_infrared == 4:
                # Turn left in episodes
                self.motor.set_motor_model(1250, 1250, -1250,-1250)  # Turn left
                time.sleep(0.15)  # Turn for a short time
                self.motor.set_motor_model(0,0,0,0)  # Stop
                time.sleep(0.1)  # Pause to check sensors

            elif right_infrared == 1:
                # Turn right in episodes
                self.motor.set_motor_model(-1250, -1250, 1250,1250)  # Turn right
                time.sleep(0.15)  # Turn for a short time
                self.motor.set_motor_model(0,0,0,0)  # Stop
                time.sleep(0.1)  # Pause to check sensors

            else:
                # Line lost, move backward in episodes
                print("Line lost, moving back...")
                self.motor.set_motor_model(-600, -600, -600, -600)  # Move backward
                time.sleep(0.2)  # Move for a short time
                self.motor.set_motor_model(0,0,0,0)  # Stop
                time.sleep(0.1)  # Pause to check sensors

    def mode_light(self):
        if (time.time() - self.car_record_time) > 0.2:
            self.car_record_time = time.time()
            self.motor.set_motor_model(0,0,0,0)
            L = self.adc.read_adc(0)
            R = self.adc.read_adc(1)
            #print("L: {}, R: {}".format(L, R))
            if L < 2.99 and R < 2.99 :
                self.motor.set_motor_model(600,600,600,600)
            elif abs(L-R)<0.15:
                self.motor.set_motor_model(0,0,0,0)
            elif L > 3 or R > 3:
                if L > R :
                    self.motor.set_motor_model(-1200,-1200,1400,1400)
                elif R > L :
                    self.motor.set_motor_model(1400,1400,-1200,-1200)

    def mode_rotate(self, n):
        angle = n
        bat_compensate = 7.5 / (self.adc.read_adc(2) * (3 if self.adc.pcb_version == 1 else 2))
        while True:
            W = 2000
            VY = int(2000 * math.cos(math.radians(angle)))
            VX = -int(2000 * math.sin(math.radians(angle)))
            FR = VY - VX + W
            FL = VY + VX - W
            BL = VY - VX - W
            BR = VY + VX + W
            print("rotating")
            self.motor.set_motor_model(FL, BL, FR, BR)
            time.sleep(5*self.time_compensate*bat_compensate/1000)
            angle -= 5

def test_car_sonic():
    car = Car()
    try:
        while True:
            car.mode_ultrasonic()
    except KeyboardInterrupt:
        car.close()
        print("\nEnd of program")

def test_car_infrared():
    car = Car()
    try:
        while True:
            car.mode_infrared()
    except KeyboardInterrupt:
        car.close()
        print("\nEnd of program")

def test_car_light():
    car = Car()
    try:
        print("Program is starting...")
        while True:
            car.mode_light()
    except KeyboardInterrupt:
        car.close()
        print("\nEnd of program")

def test_car_rotate():
    car = Car()
    print("Program is starting...")
    try:
        car.mode_rotate(0)
    except KeyboardInterrupt:
        print ("\nEnd of program")
        car.motor.set_motor_model(0,0,0,0)
        car.close()

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        print("Parameter error: Please assign the device")
        exit()
    if sys.argv[1] == 'Sonic' or sys.argv[1] == 'sonic':
        test_car_sonic()
    elif sys.argv[1] == 'Infrared' or sys.argv[1] == 'infrared':
        test_car_infrared()
    elif sys.argv[1] == 'Light' or sys.argv[1] == 'light':
        test_car_light()
    elif sys.argv[1] == 'Rotate' or sys.argv[1] == 'rotate':
        test_car_rotate()
