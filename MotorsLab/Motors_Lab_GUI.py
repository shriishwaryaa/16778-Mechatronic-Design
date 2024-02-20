import tkinter as tk
from tkinter import ttk
import math
import random
import time

# Define the global variables
dc_speed_label = None
servo_pos_label = None
canvas_dc = None
canvas_servo = None

dc_motor_speed = 56
dc_motor_pos = 0
servo_motor_pos = 56
stepper_motor_count = 0

dc_speed_range = 150
dc_pos_range = 360
servo_pos_range = 180
stepper_count_range = 2048

def update_dc_pos(pos):
    pos = round(float(pos), 2)
    update_dc_posomter(pos)

def update_dc_posomter(pos_value):
    angle = (pos_value / dc_pos_range) * 360  # Calculate the angle based on the speed value
    canvas_dc.delete("needle")  # Remove previous needle
    x = 150 - 100 * math.cos(math.radians(angle))  # Calculate the x-coordinate of the needle (negative to rotate other way)
    y = 150 - 100 * math.sin(math.radians(angle))  # Calculate the y-coordinate of the needle (negative to rotate other way)
    canvas_dc.create_line(150, 150, x, y, fill="red", width=3, tag="needle")  # Draw the needle

def update_dc_speed(speed):
    speed = round(float(speed), 2)
    update_dc_speedometer(speed)

def update_dc_speedometer(speed_value):
    percent = (speed_value) * 100 / dc_speed_range  # Calculate the angle based on the speed value
    x1 = 400  # Calculate the x-coordinate of the needle (negative to rotate other way)
    y1 = 150 - percent  # Calculate the y-coordinate of the needle (negative to rotate other way)
    canvas_dc.create_line(400, 150, x1, y1, fill="red", width=6, tag="needle")  # Draw the needle

def update_servo_pos(pos):
    pos = round(float(pos), 2)
    update_servo_posmeter(pos)

def update_servo_posmeter(pos_value):
    angle = (pos_value / servo_pos_range) * servo_pos_range  # Calculate the angle based on the speed value
    canvas_servo.delete("needle")  # Remove previous needle
    x = 300 - 100 * math.cos(math.radians(angle))  # Calculate the x-coordinate of the needle (negative to rotate other way)
    y = 150 - 100 * math.sin(math.radians(angle))  # Calculate the y-coordinate of the needle (negative to rotate other way)
    canvas_servo.create_line(300, 150, x, y, fill="red", width=3, tag="needle")  # Draw the needle

def update_stepper_pos(count):
    count = round(float(count)%stepper_count_range, 2)
    update_stepper_posmeter(count)

def update_stepper_posmeter(count_value):
    angle = (count_value / stepper_count_range) * 360  # Calculate the angle based on the speed value
    canvas_stepper.delete("needle")  # Remove previous needle
    x = 300 - 100 * math.cos(math.radians(angle))  # Calculate the x-coordinate of the needle (negative to rotate other way)
    y = 150 - 100 * math.sin(math.radians(angle))  # Calculate the y-coordinate of the needle (negative to rotate other way)
    canvas_stepper.create_line(300, 150, x, y, fill="red", width=3, tag="needle")  # Draw the needle

def update_values(root):
    global dc_motor_speed, servo_motor_pos
    dc_motor_speed = get_dc_speed()
    dc_motor_pos = get_dc_pos()
    servo_motor_pos = get_servo_position()
    stepper_motor_count = get_stepper_count()

    update_dc_pos(dc_motor_pos)
    update_servo_pos(servo_motor_pos)
    update_stepper_pos(stepper_motor_count)
    update_dc_speed(dc_motor_speed)

    root.after(500, update_values, root)  # Schedule next update after 3 seconds

def main():
    global canvas_dc, canvas_servo, canvas_stepper, dc_speed_label, servo_pos_label, stepper_count_label
    root = tk.Tk()
    root.title("Motor Lab GUI")

    canvas_dc = tk.Canvas(root, width=600, height=300, bg="white")
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

    canvas_servo = tk.Canvas(root, width=600, height=200, bg="white")
    servo_label = tk.Label(root, text = "SERVO MOTOR Position")
    servo_label.place(x=300, y=300, anchor = tk.CENTER)
    canvas_servo.pack()

    # dc_motor_speed = get_dc_speed()

    # update_dc_speed(dc_motor_speed)
    # update_servo_pos(servo_motor_pos)

    canvas_servo.create_arc(200, 50, 400, 250, start=0, extent=180, outline="black", width=3)

    for i, percent in enumerate([0, servo_pos_range/4, servo_pos_range/2, 3*servo_pos_range/4, servo_pos_range]):
        angle = (percent / servo_pos_range) * 180 - 180
        x1 = 300 + 100 * math.cos(math.radians(angle))
        y1 = 150 + 100 * math.sin(math.radians(angle))
        x2 = 300 + 120 * math.cos(math.radians(angle))
        y2 = 150 + 120 * math.sin(math.radians(angle))
        canvas_servo.create_line(x1, y1, x2, y2, fill="black", width=2)
        label_x = 300 + 125 * math.cos(math.radians(angle))
        label_y = 450 + 125 * math.sin(math.radians(angle))
        label = tk.Label(root, text=str(percent), bg="white", bd=0, highlightthickness=0)
        label.place(x=label_x, y=label_y, anchor=tk.CENTER)

    canvas_stepper = tk.Canvas(root, width=600, height=300, bg="white")
    stepper_label = tk.Label(root, text = "STEPPER MOTOR Position")
    stepper_label.place(x=300, y=520, anchor = tk.CENTER)
    canvas_stepper.pack()

    canvas_stepper.create_oval(200, 50, 400, 250, outline="black", width=3)

    for i, percent in enumerate([0, 45, 90, 135, 180, 225, 270, 315]):
        angle = (percent / 360) * 360 - 180
        x1 = 300 + 100 * math.cos(math.radians(angle))
        y1 = 150 + 100 * math.sin(math.radians(angle))
        x2 = 300 + 120 * math.cos(math.radians(angle))
        y2 = 150 + 120 * math.sin(math.radians(angle))
        canvas_stepper.create_line(x1, y1, x2, y2, fill="black", width=2)
        label_x = 300 + 125 * math.cos(math.radians(angle))
        label_y = 665 + 125 * math.sin(math.radians(angle))
        label = tk.Label(root, text=str(percent), bg="white", bd=0, highlightthickness=0)
        label.place(x=label_x, y=label_y, anchor=tk.CENTER)

    root.after(500, update_values, root)  # Schedule the initial update after 3 seconds

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

if __name__ == "__main__":
    main()
