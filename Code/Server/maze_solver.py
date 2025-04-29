from ultrasonic import Ultrasonic
from motor import Ordinary_Car
from servo import Servo
from infrared import Infrared
from adc import ADC
import time
import math
import curses
from astar import astar

MAZE_MAP = [
    [0, 1, 0, 0, 0, 1, 0, 1, 0, 0],
    [0, 1, 1, 0, 1, 1, 0, 1, 1, 0],
    [0, 0, 0, 1, 0, 0, 0, 0, 1, 0],
    [1, 0, 1, 1, 0, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
]


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

    def mode_infrared(self, counter):
        if (time.time() - self.car_record_time) > 0.2:
            self.car_record_time = time.time()
            infrared_value = self.infrared.read_all_infrared()
            print("infrared_value: " + str(infrared_value))

            left_infrared = self.infrared.read_one_infrared(1) << 2
            right_infrared = self.infrared.read_one_infrared(3)
            center_infrared = self.infrared.read_one_infrared(2) << 1

            print("left_infrared: " + str(left_infrared), "right_infrared: " + str(right_infrared), "center_infrared: " + str(center_infrared))

            if left_infrared == 4:
                counter = 0
                # Turn left in episodes
                self.motor.set_motor_model(-1250, -1250, 1250,1250)  # Turn left
                time.sleep(0.15)  # Turn for a short time
                self.motor.set_motor_model(0,0,0,0)  # Stop
                time.sleep(0.1)  # Pause to check sensors

            elif right_infrared == 1:
                counter = 0
                # Turn right in episodes
                self.motor.set_motor_model(1250, 1250, -1250,-1250)  # Turn right
                time.sleep(0.15)  # Turn for a short time
                self.motor.set_motor_model(0,0,0,0)  # Stop
                time.sleep(0.1)  # Pause to check sensors

            elif center_infrared == 2:
                counter = 0
                # Move forward in episodes
                self.motor.set_motor_model(800,800,800,800)  # Move forward
                time.sleep(0.2)  # Move for a short time
                self.motor.set_motor_model(0,0,0,0)  # Stop
                time.sleep(0.1)  # Pause to check sensors

            else:
                counter += 1
                # Line lost, move backward in episodes
                print("Line lost, moving back...")
                if counter < 10:
                    self.motor.set_motor_model(-800, -800, -800, -800)  # Move backward
                    time.sleep(0.1)  # Move backward briefly
                    self.motor.set_motor_model(0, 0, 0, 0)  # Stop
                else:
                    # Pick a random direction
                    import random
                    direction = random.choice(["left", "right"])
                    print(f"Trying random direction: {direction}")
                    if direction == "left":
                        self.motor.set_motor_model(-1250, -1250, 1250, 1250)  # Turn left
                    else:
                        self.motor.set_motor_model(1250, 1250, -1250, -1250)  # Turn right
                    time.sleep(0.2)  # Turn for a short time
                    self.motor.set_motor_model(800, 800, 800, 800)  # Move forward
                time.sleep(0.1)  # Pause to check sensors

    def mode_solve_maze(self):
        if (time.time() - self.car_record_time) > 0.2:
            self.car_record_time = time.time()


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

    def execute_path(car, path):
        steps_taken = 0
        for (cur_x, cur_y), (next_x, next_y) in zip(path, path[1:]):
            dx = next_x - cur_x
            dy = next_y - cur_y

            if dx == 1:  # move down
                car.motor.set_motor_model(800,800,800,800)
            elif dx == -1:  # move up
                car.motor.set_motor_model(-800,-800,-800,-800)
            elif dy == 1:  # move right
                car.motor.set_motor_model(1250,1250,-1250,-1250)
            elif dy == -1:  # move left
                car.motor.set_motor_model(-1250,-1250,1250,1250)

            steps_taken += 1
            time.sleep(0.5)  # Assume 0.5s = 1 cell block
            car.motor.set_motor_model(0,0,0,0)
            time.sleep(0.2)

        print(f"Total blocks moved: {steps_taken}")

    def manual_control(self, stdscr):
        # Set up curses for manual control
        stdscr.nodelay(True)  # Non-blocking input
        curses.curs_set(0)    # Hide cursor
        stdscr.clear()
        stdscr.addstr(0, 0, "Manual Control Mode - WASD keys to move (Q to return to menu)")
        stdscr.refresh()

        # Movement tracking
        last_key_time = 0
        current_key = None

        try:
            while True:
                key = stdscr.getch()
                current_time = time.time()

                # Always register new key presses
                if key != -1:
                    current_key = key
                    last_key_time = current_time

                    if key == ord('q'):
                        return  # Return to main menu

                    # Process movement commands
                    if key == ord('w'):  # Forward
                        stdscr.addstr(1, 0, "Moving forward  ")
                        self.motor.set_motor_model(800, 800, 800, 800)
                    elif key == ord('s'):  # Backward
                        stdscr.addstr(1, 0, "Moving backward ")
                        self.motor.set_motor_model(-800, -800, -800, -800)
                    elif key == ord('a'):  # Left
                        stdscr.addstr(1, 0, "Turning left    ")
                        self.motor.set_motor_model(-1250, -1250, 1250, 1250)
                    elif key == ord('d'):  # Right
                        stdscr.addstr(1, 0, "Turning right   ")
                        self.motor.set_motor_model(1250, 1250, -1250, -1250)
                    elif key == ord(' '):  # Space to explicitly stop
                        stdscr.addstr(1, 0, "Stopped         ")
                        self.motor.set_motor_model(0, 0, 0, 0)
                        current_key = None

                # Auto-stop if no key press for 0.1 seconds
                if current_key is not None and current_time - last_key_time > 0.1:
                    stdscr.addstr(1, 0, "Stopped (auto)   ")
                    self.motor.set_motor_model(0, 0, 0, 0)
                    current_key = None

                # Debug info
                stdscr.addstr(2, 0, f"Key: {chr(current_key) if current_key else 'None'} | Time since: {current_time - last_key_time:.3f}s")
                stdscr.refresh()

                # Very small delay for CPU efficiency but maintain responsiveness
                time.sleep(0.01)

        finally:
            # Make sure motors are stopped when exiting manual mode
            self.motor.set_motor_model(0, 0, 0, 0)

    def display_menu(self, stdscr):
        # Set up curses for menu display
        curses.curs_set(1)  # Show cursor
        stdscr.nodelay(False)  # Blocking input for menu
        stdscr.clear()

        # Menu options
        menu_items = [
            "1. Manual Control (WASD)",
            "2. Line Tracking",
            "q. Quit"
        ]

        # Display menu
        stdscr.addstr(0, 0, "Robot Control Menu")
        stdscr.addstr(1, 0, "-----------------")
        for i, item in enumerate(menu_items):
            stdscr.addstr(i+3, 0, item)

        stdscr.addstr(len(menu_items)+4, 0, "Enter your choice: ")
        stdscr.refresh()

        return stdscr.getch()

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
        counter = 0
        while True:
            car.mode_infrared(counter)
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

def test_solve_maze():
    car = Car()
    try:
        car.mode_solve_maze()
    except KeyboardInterrupt:
        car.close()
        print("\nEnd of program")


def test_car_astar():
    car = Car()
    try:
        start = (0, 0)
        goal = (4, 9)
        path = astar(MAZE_MAP, start, goal)
        print("Path found by A*: ", path)
        car.execute_path(path)
    except KeyboardInterrupt:
        print("Execution interrupted.")
    finally:
        car.close()

if __name__ == '__main__':
    import sys
    if len(sys.argv) < 2:
        car = Car()
        try:
            # Initialize curses
            stdscr = curses.initscr()
            curses.noecho()
            curses.cbreak()

            while True:
                choice = car.display_menu(stdscr)

                if choice == ord('1'):
                    car.manual_control(stdscr)
                elif choice == ord('2'):
                    try:
                        while True:
                            car.mode_infrared()
                    except KeyboardInterrupt:
                        car.close()
                        print("\nEnd of program")
                elif choice == ord('q'):
                    break

        except KeyboardInterrupt:
            pass
        finally:
            # Clean up curses
            curses.nocbreak()
            curses.echo()
            curses.endwin()
            car.close()
    else:
        if sys.argv[1] == 'Sonic' or sys.argv[1] == 'sonic':
            test_car_sonic()
        elif sys.argv[1] == 'Infrared' or sys.argv[1] == 'infrared':
            test_car_infrared()
        elif sys.argv[1] == 'Light' or sys.argv[1] == 'light':
            test_car_light()
        elif sys.argv[1] == 'Rotate' or sys.argv[1] == 'rotate':
            test_car_rotate()
        elif sys.argv[1].lower() == 'astar':
            test_car_astar()
        elif sys.argv[1].lower() == 'maze':
            test_solve_maze()
