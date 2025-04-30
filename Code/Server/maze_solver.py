from ultrasonic import Ultrasonic
from motor import Ordinary_Car
from servo import Servo
from infrared import Infrared
from adc import ADC
import time
import math
import curses
from astar import astar
import random

MAZE_MAP = {
    (0, 0): [(1, 0), (0, 1)],
    (0, 1): [(0, 0), (0, 2)],
    (0, 2): [(0, 1), (1, 2)],
    (0, 3): [(1, 3), (0, 4)],
    (0, 4): [(0, 3), (0, 5)],
    (0, 5): [(0, 4), (1, 5)],
    (0, 6): [(1, 6), (0, 7)],
    (0, 7): [(0, 6), (0, 8)],
    (0, 8): [(0, 7), (0, 9)],
    (0, 9): [(0, 8), (1, 9)],

    (1, 0): [(0, 0), (2, 0)],
    (1, 1): [(1, 2)],
    (1, 2): [(1, 1), (0, 2)],
    (1, 3): [(0, 3), (1, 4)],
    (1, 4): [(1, 3), (2, 4)],
    (1, 5): [(0, 5), (1, 6)],
    (1, 6): [(0, 6), (2, 6), (1, 5)],
    (1, 7): [(2, 7)],
    (1, 8): [(2, 8), (1, 9)],
    (1, 9): [(0, 9), (1, 8)],

    (2, 0): [(1, 0), (2, 1)],
    (2, 1): [(2, 0), (3, 1)],
    (2, 2): [(3, 2), (2, 3)],
    (2, 3): [(2, 2)],
    (2, 4): [(1, 4), (2, 5), (3, 4)],
    (2, 5): [(2, 4), (3, 5)],
    (2, 6): [(1, 6), (2, 7)],
    (2, 7): [(2, 6), (1, 7)],
    (2, 8): [(3, 8), (1, 8)],
    (2, 9): [(3, 9)],

    (3, 0): [(4, 0), (3, 1)],
    (3, 1): [(3, 0), (2, 1), (3, 2)],
    (3, 2): [(3, 1), (2, 2)],
    (3, 3): [(4, 3), (3, 4)],
    (3, 4): [(3, 3), (2, 4), (4, 4)],
    (3, 5): [(2, 5), (3, 6)],
    (3, 6): [(3, 5)],
    (3, 7): [(4, 7), (3, 8)],
    (3, 8): [(3, 7), (3, 9), (2, 8)],
    (3, 9): [(3, 8), (2, 9)],

    (4, 0): [(3, 0), (4, 1)],
    (4, 1): [(4, 0), (4, 2)],
    (4, 2): [(4, 1), (4, 3)],
    (4, 3): [(4, 2), (3, 3)],
    (4, 4): [(3, 4), (4, 5)],
    (4, 5): [(4, 4)],
    (4, 6): [(4, 7)],
    (4, 7): [(4, 6), (4, 8)],
    (4, 8): [(4, 7), (4, 9)],
    (4, 9): [(4, 8)]
}


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
                counter[0] = 0
                # Turn left in episodes
                self.motor.set_motor_model(-1250, -1250, 1250,1250)  # Turn left
                time.sleep(0.15)  # Turn for a short time
                self.motor.set_motor_model(0,0,0,0)  # Stop
                time.sleep(0.1)  # Pause to check sensors

            elif right_infrared == 1:
                counter[0] = 0
                # Turn right in episodes
                self.motor.set_motor_model(1250, 1250, -1250,-1250)  # Turn right
                time.sleep(0.15)  # Turn for a short time
                self.motor.set_motor_model(0,0,0,0)  # Stop
                time.sleep(0.1)  # Pause to check sensors

            elif center_infrared == 2:
                counter[0] = 0
                # Move forward in episodes
                self.motor.set_motor_model(800,800,800,800)  # Move forward
                time.sleep(0.2)  # Move for a short time
                self.motor.set_motor_model(0,0,0,0)  # Stop
                time.sleep(0.1)  # Pause to check sensors

            else:
                print('counter', counter[0])
                counter[0] += 1
                # Line lost, move backward in episodes
                print("Line lost, moving back...")

                chance = random.randint(0, 100)


                if chance < 10:
                    direction = random.choice(["left", "right"])
                    print(f"Trying random direction: {direction}")
                    if direction == "left":
                        self.motor.set_motor_model(-1250, -1250, 1250, 1250)  # Turn left
                    else:
                        self.motor.set_motor_model(1250, 1250, -1250, -1250)  # Turn right
                    time.sleep(0.2)  # Turn for a short time
                    self.motor.set_motor_model(800, 800, 800, 800)  # Move forward


                if counter[0] < 10:
                    self.motor.set_motor_model(-800, -800, -800, -800)  # Move backward
                    time.sleep(0.1)  # Move backward briefly
                    self.motor.set_motor_model(0, 0, 0, 0)  # Stop
                else:
                    # Pick a random direction

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

    def execute_path_graph(self, path):
        steps_taken = 0
        step_size = 0.5  # seconds for forward/backward movement
        turn_size = 0.3  # seconds for turning
        pause_size = 0.2  # seconds to pause between movements

        direction_map = {
            (1, 0): "forward",
            (-1, 0): "backward",
            (0, 1): "left",
            (0, -1): "right"
        }

        for (cur_x, cur_y), (next_x, next_y) in zip(path, path[1:]):
            dx = next_x - cur_x
            dy = next_y - cur_y
            direction = direction_map.get((dx, dy))

            print(f"Moving from {(cur_x, cur_y)} to {(next_x, next_y)}: {direction}")

            if direction == "forward":
                # Move forward
                self.motor.set_motor_model(800, 800, 800, 800)
                time.sleep(step_size)
                self.motor.set_motor_model(0, 0, 0, 0)
                time.sleep(pause_size)

            elif direction == "backward":
                # Move backward
                self.motor.set_motor_model(-800, -800, -800, -800)
                time.sleep(step_size)
                self.motor.set_motor_model(0, 0, 0, 0)
                time.sleep(pause_size)

            elif direction == "left":
                # First turn left
                self.motor.set_motor_model(-1250, -1250, 1250, 1250)
                time.sleep(turn_size)
                self.motor.set_motor_model(0, 0, 0, 0)
                time.sleep(pause_size)

                # Then move forward
                self.motor.set_motor_model(800, 800, 800, 800)
                time.sleep(step_size)
                self.motor.set_motor_model(0, 0, 0, 0)
                time.sleep(pause_size)

            elif direction == "right":
                # First turn right
                self.motor.set_motor_model(1250, 1250, -1250, -1250)
                time.sleep(turn_size)
                self.motor.set_motor_model(0, 0, 0, 0)
                time.sleep(pause_size)

                # Then move forward
                self.motor.set_motor_model(800, 800, 800, 800)
                time.sleep(step_size)
                self.motor.set_motor_model(0, 0, 0, 0)
                time.sleep(pause_size)

            steps_taken += 1
            print(f"Completed step {steps_taken}")

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
        counter = [0]
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
        start = (0, 0)  # Update if your robot starts elsewhere
        goal = (4, 9)   # Update if your goal changes
        path = astar(MAZE_MAP, start, goal)
        print("Path found:", path)
        car.execute_path_graph(path)
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
