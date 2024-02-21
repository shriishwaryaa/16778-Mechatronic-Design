import tkinter as tk
from tkinter import ttk
import math
import random
import time
import serial

# Define the global variables
dc_speed_label = None
servo_pos_label = None
canvas_dc = None
canvas_servo = None

delay_time = 100 #ms

#inputs
dc_motor_speed = 56
dc_motor_pos = 0
servo_motor_pos = 56
stepper_motor_count = 0
ultrasonic_reading = 0
temperature_reading = 0
pot1_reading = 0
pot2_reading = 0
pot3_reading = 0

#outputs
desired_code_state = 0
desired_servo_pos = 0
desired_motor_pos = 0
desired_motor_vel = 0
desired_stepper_turn = 0

#ranges
dc_speed_range = 100
dc_pos_range = 360
servo_pos_range = 180
stepper_count_range = 2048
ultrasonic_range = 400 #cm
temperature_range = 100
pot1_range = 10000 #ohms

def update_dc_pos(pos):
    pos = round(float(pos), 2)
    update_dc_posometer(pos)

def update_dc_posometer(pos_value):
    angle = (pos_value / dc_pos_range) * 360  # Calculate the angle based on the speed value
    canvas_dc.delete("needle")  # Remove previous needle
    x = 150 - 100 * math.cos(math.radians(angle))  # Calculate the x-coordinate of the needle (negative to rotate other way)
    y = 150 - 100 * math.sin(math.radians(angle))  # Calculate the y-coordinate of the needle (negative to rotate other way)
    canvas_dc.create_line(150, 150, x, y, fill="blue", width=3, tag="needle")  # Draw the needle

def update_dc_speed(speed):
    speed = round(float(speed), 2)
    update_dc_speedometer(speed)

def update_dc_speedometer(speed_value):
    percent = (speed_value) * 100 / dc_speed_range  # Calculate the angle based on the speed value
    x1 = 400  # Calculate the x-coordinate of the needle (negative to rotate other way)
    y1 = 150 - percent  # Calculate the y-coordinate of the needle (negative to rotate other way)
    canvas_dc.create_line(400, 150, x1, y1, fill="blue", width=6, tag="needle")  # Draw the needle

def update_ultrasonic(dist):
    dist = round(float(dist), 2)
    update_ultrasonic_monitor(dist)

def update_ultrasonic_monitor(dist_value):
    percent = (dist_value) * 100 / ultrasonic_range  # Calculate the angle based on the speed value
    x1 = 375  # Calculate the x-coordinate of the needle (negative to rotate other way)
    y1 = 150 - percent  # Calculate the y-coordinate of the needle (negative to rotate other way)
    canvas_servo.create_line(375, 150, x1, y1, fill="blue", width=6, tag="needle")  # Draw the needle

def update_temp(temp):
    temp = round(float(temp), 2)
    update_thermometer(temp)

def update_thermometer(temperature):
    percent = (temperature) * 100 / temperature_range  # Calculate the angle based on the speed value
    x1 = 525  # Calculate the x-coordinate of the needle (negative to rotate other way)
    y1 = 150 - percent  # Calculate the y-coordinate of the needle (negative to rotate other way)
    if(temperature>50):
        canvas_servo.create_line(525, 150, x1, y1, fill="red", width=6, tag="needle")  # Draw the needle
    else:
        canvas_servo.create_line(525, 150, x1, y1, fill="green", width=6, tag="needle")  # Draw the needle

def update_servo_pos(pos):
    pos = round(float(pos), 2)
    update_servo_posmeter(pos)

def update_servo_posmeter(pos_value):
    angle = (pos_value / servo_pos_range) * servo_pos_range  # Calculate the angle based on the speed value
    canvas_servo.delete("needle")  # Remove previous needle
    x = 150 - 100 * math.cos(math.radians(angle))  # Calculate the x-coordinate of the needle (negative to rotate other way)
    y = 150 - 100 * math.sin(math.radians(angle))  # Calculate the y-coordinate of the needle (negative to rotate other way)
    canvas_servo.create_line(150, 150, x, y, fill="blue", width=3, tag="needle")  # Draw the needle

def update_pot_pos(ohm):
    ohm = round(float(ohm), 2)
    update_pot(ohm)

def update_pot(ohm_value):
    angle = (ohm_value / pot1_range) * pot1_range  # Calculate the angle based on the speed value
    canvas_stepper.delete("needle")  # Remove previous needle
    x = 450 - 100 * math.cos(math.radians(angle))  # Calculate the x-coordinate of the needle (negative to rotate other way)
    y = 150 - 100 * math.sin(math.radians(angle))  # Calculate the y-coordinate of the needle (negative to rotate other way)
    canvas_stepper.create_line(450, 150, x, y, fill="blue", width=3, tag="needle")  # Draw the needle

def update_stepper_pos(count):
    count = round(float(count)%stepper_count_range, 2)
    update_stepper_posmeter(count)

def update_stepper_posmeter(count_value):
    angle = (count_value / stepper_count_range) * 360  # Calculate the angle based on the speed value
    canvas_stepper.delete("needle1")  # Remove previous needle
    x = 150 - 100 * math.cos(math.radians(angle))  # Calculate the x-coordinate of the needle (negative to rotate other way)
    y = 150 - 100 * math.sin(math.radians(angle))  # Calculate the y-coordinate of the needle (negative to rotate other way)
    canvas_stepper.create_line(150, 150, x, y, fill="blue", width=3, tag="needle1")  # Draw the needle

def update_values(root):
    global dc_motor_speed, servo_motor_pos
    dc_motor_speed, dc_motor_pos, servo_motor_pos, stepper_motor_count, ultrasonic_reading, temperature_reading, pot1_reading = read_from_arduino()

    update_dc_pos(dc_motor_pos)
    update_servo_pos(servo_motor_pos)
    update_stepper_pos(stepper_motor_count)
    update_dc_speed(dc_motor_speed)
    update_ultrasonic(ultrasonic_reading)
    update_temp(temperature_reading)
    update_pot(pot1_reading)

    desired_code_state = (desired_state_entry.get())
    desired_servo_pos = (desired_servo_pos_entry.get())
    desired_motor_pos = (desired_motor_pos_entry.get())
    desired_motor_vel = (desired_motor_vel_entry.get())
    desired_stepper_turn = (desired_stepper_turn_entry.get())

    write_to_arduino(desired_code_state, desired_servo_pos, desired_motor_pos, desired_motor_vel, desired_stepper_turn)

    root.after(delay_time, update_values, root)  # Schedule next update after 3 seconds

def write_to_arduino(code_state, servo_pos, motor_pos, motor_vel, stepper_turn):
    str = (code_state, servo_pos, motor_pos, motor_vel, stepper_turn) 
    print(str)
    arduino.write(str)

def read_from_arduino():
    data = arduino.readLine()
    return data

def main():
    global canvas_dc, canvas_servo, canvas_stepper, dc_speed_label, servo_pos_label, stepper_count_label
    global desired_servo_pos, desired_motor_pos, desired_motor_vel, desired_stepper_turn
    global desired_state_entry, desired_servo_pos_entry, desired_motor_pos_entry, desired_motor_vel_entry, desired_stepper_turn_entry
    root = tk.Tk()
    root.title("Motor Lab GUI")

    canvas_dc = tk.Canvas(root, width=800, height=300, bg="white")
    dc_label = tk.Label(root, text = "DC MOTOR Position (Left) + Speed (Right)")
    dc_label.place(x=300, y=20, anchor = tk.CENTER)
    canvas_dc.pack()

    canvas_dc.create_oval(50, 50, 250, 250, outline="black", width=3)

    for i, percent in enumerate([0, 45, 90, 135, 180, 225, 270, 315]):
        angle = (percent / 360) * 360 - 180
        x1 = 150 + 100 * math.cos(math.radians(angle))
        y1 = 150 + 100 * math.sin(math.radians(angle))
        x2 = 150 + 120 * math.cos(math.radians(angle))
        y2 = 150 + 120 * math.sin(math.radians(angle))
        canvas_dc.create_line(x1, y1, x2, y2, fill="black", width=2)
        label_x = 150 + 125 * math.cos(math.radians(angle))
        label_y = 150 + 125 * math.sin(math.radians(angle))
        label = tk.Label(root, text=str(percent), bd=0, highlightthickness=0)
        label.place(x=label_x, y=label_y, anchor=tk.CENTER)

    for i, percent in enumerate([-1*dc_speed_range, -3*dc_speed_range/4, -1*dc_speed_range/2, -1*dc_speed_range/4, 0, dc_speed_range/4, dc_speed_range/2, 3*dc_speed_range/4, dc_speed_range]):
        x1 = 397
        y1 = 150 - (percent) * 100 / dc_speed_range
        x2 = 410
        y2 = y1
        canvas_dc.create_line(x1, y1, x2, y2, fill="black", width=2)
        label_x = 412
        label_y = y1
        label = tk.Label(root, text=str(percent) + " rpm", bd=0)
        label.place(x=label_x, y=label_y, anchor=tk.W)

    canvas_servo = tk.Canvas(root, width=800, height=200, bg="white")
    servo_label = tk.Label(root, text = "SERVO MOTOR Position (deg)")
    ultrasonic_label = tk.Label(root, text="Ultrasonic reading")
    temp_label = tk.Label(root, text="Temperature reading")
    servo_label.place(x=150, y=300, anchor = tk.CENTER)
    ultrasonic_label.place(x=375, y=325, anchor=tk.CENTER)
    temp_label.place(x=525, y=325, anchor=tk.CENTER)
    canvas_servo.pack()

    # User input boxes
    desired_state_entry = tk.Entry(root)
    desired_state_label = tk.Label(root, text="Enter Desired Code State")
    desired_state_label.place(x=700, y=30, anchor=tk.CENTER)
    desired_state_entry.place(x=700, y=50, anchor=tk.CENTER)
    desired_state_entry.insert(0, str(desired_code_state))

    desired_servo_pos_entry = tk.Entry(root)
    desired_servo_label = tk.Label(root, text="Enter Desired Servo Position")
    desired_servo_label.place(x=700, y=80, anchor=tk.CENTER)
    desired_servo_pos_entry.place(x=700, y=100, anchor=tk.CENTER)
    desired_servo_pos_entry.insert(0, str(desired_servo_pos))

    desired_motor_pos_entry = tk.Entry(root)
    desired_motor_pos_label = tk.Label(root, text="Enter Desired Motor Position")
    desired_motor_pos_label.place(x=700, y=130, anchor=tk.CENTER)
    desired_motor_pos_entry.place(x=700, y=150, anchor=tk.CENTER)
    desired_motor_pos_entry.insert(0, str(desired_motor_pos))

    desired_motor_vel_entry = tk.Entry(root)
    desired_motor_vel_label = tk.Label(root, text="Enter Desired Motor Velocity")
    desired_motor_vel_label.place(x=700, y=180, anchor=tk.CENTER)
    desired_motor_vel_entry.place(x=700, y=200, anchor=tk.CENTER)
    desired_motor_vel_entry.insert(0, str(desired_motor_vel))

    desired_stepper_turn_entry = tk.Entry(root)
    desired_stepper_label = tk.Label(root, text="Enter Desired Stepper Position")
    desired_stepper_label.place(x=700, y=230, anchor=tk.CENTER)
    desired_stepper_turn_entry.place(x=700, y=250, anchor=tk.CENTER)
    desired_stepper_turn_entry.insert(0, str(desired_stepper_turn))


    # dc_motor_speed = get_dc_speed()

    # update_dc_speed(dc_motor_speed)
    # update_servo_pos(servo_motor_pos)

    canvas_servo.create_arc(50, 50, 250, 250, start=0, extent=180, outline="black", width=3)

    for i, percent in enumerate([0, servo_pos_range/4, servo_pos_range/2, 3*servo_pos_range/4, servo_pos_range]):
        angle = (percent / servo_pos_range) * 180 - 180
        x1 = 150 + 100 * math.cos(math.radians(angle))
        y1 = 150 + 100 * math.sin(math.radians(angle))
        x2 = 150 + 120 * math.cos(math.radians(angle))
        y2 = 150 + 120 * math.sin(math.radians(angle))
        canvas_servo.create_line(x1, y1, x2, y2, fill="black", width=2)
        label_x = 150 + 125 * math.cos(math.radians(angle))
        label_y = 450 + 125 * math.sin(math.radians(angle))
        label = tk.Label(root, text=str(percent), bg="white", bd=0, highlightthickness=0)
        label.place(x=label_x, y=label_y, anchor=tk.CENTER)

    for i, percent in enumerate([0, ultrasonic_range/4, ultrasonic_range/2, 3*ultrasonic_range/4, ultrasonic_range]):
        x1 = 372
        y1 = 150 - (percent) * 100 / ultrasonic_range
        x2 = 383
        y2 = y1
        canvas_servo.create_line(x1, y1, x2, y2, fill="black", width=2)
        label_x = 385
        label_y = y1+305
        label = tk.Label(root, text=str(percent) + " cm", bd=0)
        label.place(x=label_x, y=label_y, anchor=tk.W)

    for i, percent in enumerate([0, temperature_range/4, temperature_range/2, 3*temperature_range/4, temperature_range]):
        x1 = 522
        y1 = 150 - (percent) * 100 / temperature_range
        x2 = 530
        y2 = y1
        if percent == temperature_range/2:
            x1 = 517
            canvas_servo.create_line(x1, y1, x2, y2, fill="blue", width=2)
        canvas_servo.create_line(x1, y1, x2, y2, fill="black", width=2)
        label_x = 532
        label_y = y1+305
        label = tk.Label(root, text=str(percent) + " deg F", bd=0)
        label.place(x=label_x, y=label_y, anchor=tk.W)

    canvas_stepper = tk.Canvas(root, width=800, height=300, bg="white")
    stepper_label = tk.Label(root, text = "STEPPER MOTOR Position (Deg)")
    pot_label = tk.Label(root, text="Potentiometer resistance (Ohms)")
    stepper_label.place(x=150, y=520, anchor = tk.CENTER)
    pot_label.place(x=450, y=520, anchor=tk.CENTER)
    canvas_stepper.pack()

    canvas_stepper.create_oval(50, 50, 250, 250, outline="black", width=3)
    canvas_stepper.create_oval(350, 50, 550, 250, outline="black", width=3)

    for i, percent in enumerate([0, 45, 90, 135, 180, 225, 270, 315]):
        angle = (percent / 360) * 360 - 180
        x1 = 150 + 100 * math.cos(math.radians(angle))
        y1 = 150 + 100 * math.sin(math.radians(angle))
        x2 = 150 + 120 * math.cos(math.radians(angle))
        y2 = 150 + 120 * math.sin(math.radians(angle))
        canvas_stepper.create_line(x1, y1, x2, y2, fill="black", width=2)
        label_x = 150 + 125 * math.cos(math.radians(angle))
        label_y = 665 + 125 * math.sin(math.radians(angle))
        label = tk.Label(root, text=str(percent), bg="white", bd=0, highlightthickness=0)
        label.place(x=label_x, y=label_y, anchor=tk.CENTER)

    for i, percent in enumerate([0, pot1_range/8, 2*pot1_range/8, 3*pot1_range/8, pot1_range/2, 5*pot1_range/8, 3*pot1_range/4, 7*pot1_range/8]):
        angle = (percent / pot1_range) * 360 - 180
        x1 = 450 + 100 * math.cos(math.radians(angle))
        y1 = 150 + 100 * math.sin(math.radians(angle))
        x2 = 450 + 120 * math.cos(math.radians(angle))
        y2 = 150 + 120 * math.sin(math.radians(angle))
        canvas_stepper.create_line(x1, y1, x2, y2, fill="black", width=2)
        label_x = 450 + 125 * math.cos(math.radians(angle))
        label_y = 665 + 125 * math.sin(math.radians(angle))
        label = tk.Label(root, text=str(percent), bd=0, highlightthickness=0)
        label.place(x=label_x, y=label_y, anchor=tk.CENTER)

    root.after(delay_time, update_values, root)  # Schedule the initial update after 3 seconds

    root.mainloop()

def get_dc_speed():
    r = random.randint(-1*dc_speed_range, dc_speed_range)
    return r

def get_dc_pos():
    r = random.randint(0, dc_pos_range)
    return r

def get_servo_position():
    r = random.randint(0, servo_pos_range)
    return r

def get_stepper_count():
    r = random.randint(0, 10000000)
    return r

def get_ultrasonic_dist():
    r = random.randint(0, ultrasonic_range)
    return r

def get_temperature():
    r = random.randint(0, temperature_range)
    return r

def get_resistance():
    r = random.randint(0, pot1_range)
    return r

if __name__ == "__main__":
    arduino = serial.Serial(port='COM4', baudrate=115200, timeout=.1) 
    main()
