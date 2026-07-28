# modules/drive_module.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

try:
    import afb
    _HAS_AFB = True
except Exception:
    _HAS_AFB = False

BASE_ANGLE = 78.5
BASE_SPEED = 60

# 각도 기반 결정 임계값
THETA0 = 1.5    # 직진
THETA1 = 5.0    # 보정
THETA2 = 18.0   # 최대회전

current_mode = "STOP"
current_speed = 0

def init(base_angle=None, base_speed=None):
    global BASE_ANGLE, BASE_SPEED
    if base_angle is not None: BASE_ANGLE = float(base_angle)
    if base_speed is not None: BASE_SPEED = int(base_speed)
    if _HAS_AFB: afb.gpio.init()

def get_current_mode():
    return current_mode

def get_current_speed():
    return current_speed

def forward(speed=None, angle=None):
    global current_mode, current_speed
    v = BASE_SPEED if speed is None else speed
    a = BASE_ANGLE if angle is None else angle
    if _HAS_AFB:
        afb.gpio.servo(a)
        afb.gpio.motor(80)
    current_mode = "FORWARD"
    current_speed = v

def turn_left():
    global current_mode, current_speed
    if _HAS_AFB:
        afb.gpio.servo(33)
        afb.gpio.motor(60)
    current_mode = "LEFT"
    current_speed = 50

def turn_right():
    global current_mode, current_speed
    if _HAS_AFB:
        afb.gpio.servo(147)
        afb.gpio.motor(60)
    current_mode = "RIGHT"
    current_speed = 50

def lane_adjust_left(angle=None):
    global current_mode, current_speed
    a = BASE_ANGLE if angle is None else angle
    if _HAS_AFB:
        afb.gpio.servo(a)
        afb.gpio.motor(40)
    current_mode = "LANE_ADJ_LEFT"
    current_speed = 40

def lane_adjust_right(angle=None):
    global current_mode, current_speed
    a = BASE_ANGLE if angle is None else angle
    if _HAS_AFB:
        afb.gpio.servo(a)
        afb.gpio.motor(40)
    current_mode = "LANE_ADJ_RIGHT"
    current_speed = 40

def back(speed=None):
    global current_mode, current_speed
    v = -(BASE_SPEED if speed is None else speed)
    if _HAS_AFB:
        afb.gpio.servo(BASE_ANGLE)
        afb.gpio.motor(v)
    current_mode = "BACK"
    current_speed = v

def stop():
    global current_mode, current_speed
    if _HAS_AFB:
        afb.gpio.motor(0)
    current_mode = "STOP"
    current_speed = 0

def drive_by_angle(angle):
    """차선 각도로 7가지 동작 결정. 상태 전환 시 0.5초 정지 후 동작."""
    if angle is None:
        new_mode = "STOP"
    else:
        e = float(angle) - float(BASE_ANGLE)
        if abs(e) <= THETA0:
            new_mode = "FORWARD"
        elif e <= -THETA2:
            new_mode = "LEFT"
        elif e >= THETA2:
            new_mode = "RIGHT"
        elif e < -THETA1:
            new_mode = "LANE_ADJ_LEFT"
        elif e > THETA1:
            new_mode = "LANE_ADJ_RIGHT"
        else:
            new_mode = "FORWARD"

    if new_mode != current_mode and current_mode != "STOP":
        stop()
        time.sleep(0.5)

    if new_mode == "FORWARD":
        forward()
    elif new_mode == "LEFT":
        turn_left()
    elif new_mode == "RIGHT":
        turn_right()
    elif new_mode == "LANE_ADJ_LEFT":
        lane_adjust_left(angle)
    elif new_mode == "LANE_ADJ_RIGHT":
        lane_adjust_right(angle)
    else:
        stop()
