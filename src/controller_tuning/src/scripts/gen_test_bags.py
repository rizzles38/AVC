#!/usr/bin/env python

import rospy
import rosbag
from rover12_drivers.msg import ServoControl, AutonomousMode

# Tick dt is the duration between each tick. The individual topic ticks control
# how many ticks per message on that topic.
tick_dt = 0.02
autonomous_tick = 5
servo_control_tick = 1

# Topic names.
autonomous_topic = '/control/autonomous'
servo_control_topic = '/control/command'

# Steering control.
straight_steering_us = 1556
zero_throttle_us = 1500

# Takes an integer tick number and boolean autonomous value and writes an
# AutonomousMode message into the bag on the correct topic at time t.
def write_autonomous(bag, tick, autonomous):
    msg = AutonomousMode()
    msg.header.frame_id = 'base_link'
    msg.header.stamp = rospy.Time.from_sec(tick * tick_dt)
    msg.autonomous = autonomous
    bag.write(autonomous_topic, msg, msg.header.stamp)

# Takes an integer tick number and integer throttle value and writes a
# ServoControl message into the bag on the correct topic at time t.
def write_throttle(bag, tick, throttle):
    msg = ServoControl()
    msg.header.frame_id = 'base_link'
    msg.header.stamp = rospy.Time.from_sec(tick * tick_dt)
    msg.steering_us = straight_steering_us
    msg.throttle_us = throttle
    bag.write(servo_control_topic, msg, msg.header.stamp)

def lead_in(bag, now):
    start = now
    next_auto = now
    next_servo = now
    while now < start + 200:
        if now >= next_auto:
            if now >= start + 100:
                write_autonomous(bag, now, True)
            else:
                write_autonomous(bag, now, False)
            next_auto = now + autonomous_tick

        if now >= next_servo:
            write_throttle(bag, now, zero_throttle_us)
            next_servo = now + servo_control_tick

        now += min(autonomous_tick, servo_control_tick)
    return now

def lead_out(bag, now):
    start = now
    next_auto = now
    next_servo = now
    while now < start + 100:
        if now >= next_auto:
            if now >= start + 50:
                write_autonomous(bag, now, False)
            else:
                write_autonomous(bag, now, True)
            next_auto = now + autonomous_tick

        if now >= next_servo:
            write_throttle(bag, now, zero_throttle_us)
            next_servo = now + servo_control_tick

        now += min(autonomous_tick, servo_control_tick)
    return now

def impulse(bag, now, duration, value):
    start = now
    next_auto = now
    next_servo = now
    while now < start + duration:
        if now >= next_auto:
            write_autonomous(bag, now, True)
            next_auto = now + autonomous_tick

        if now >= next_servo:
            write_throttle(bag, now, value)
            next_servo = now + servo_control_tick

        now += min(autonomous_tick, servo_control_tick)
    return now

def ramp(bag, now, duration, min_value, max_value):
    start = now
    next_auto = now
    next_servo = now
    delta_tick = (max_value - min_value) / float(duration)
    while now < start + duration:
        if now >= next_auto:
            write_autonomous(bag, now, True)
            next_auto = now + autonomous_tick

        if now >= next_servo:
            write_throttle(bag, now, min_value + (now - start) * delta_tick)
            next_servo = now + servo_control_tick

        now += min(autonomous_tick, servo_control_tick)
    return now

def throttle_impulse(duration, value):
    filename = 'throttle_impulse_{0}s_{1}.bag'.format(duration / 50, value)
    with rosbag.Bag(filename, 'w') as bag:
        now = 0
        now = lead_in(bag, now)
        now = impulse(bag, now, duration, value)
        now = lead_out(bag, now)

def throttle_ramp(duration, min_value, max_value):
    filename = 'throttle_ramp_{0}s_{1}_{2}.bag'.format(duration / 50, min_value, max_value)
    with rosbag.Bag(filename, 'w') as bag:
        now = 0
        now = lead_in(bag, now)
        now = ramp(bag, now, duration, min_value, max_value)
        now = lead_out(bag, now)

def brake_impulse(throttle_duration, throttle_value, brake_duration, brake_value):
    filename = 'brake_impulse_{0}s_{1}_{2}s_{3}.bag'.format(throttle_duration / 50, throttle_value, brake_duration / 50, brake_value)
    with rosbag.Bag(filename, 'w') as bag:
        now = 0
        now = lead_in(bag, now)
        now = impulse(bag, now, throttle_duration, throttle_value)
        now = impulse(bag, now, brake_duration, brake_value)
        now = lead_out(bag, now)

def brake_ramp(throttle_duration, throttle_value, brake_duration, brake_min, brake_max):
    filename = 'brake_ramp_{0}s_{1}_{2}s_{3}_{4}.bag'.format(throttle_duration / 50, throttle_value, brake_duration / 50, brake_min, brake_max)
    with rosbag.Bag(filename, 'w') as bag:
        now = 0
        now = lead_in(bag, now)
        now = impulse(bag, now, throttle_duration, throttle_value)
        now = ramp(bag, now, brake_duration, brake_min, brake_max)
        now = lead_out(bag, now)

def ramp_ramp(throttle_duration, throttle_min, throttle_max, brake_duration, brake_min, brake_max):
    filename = 'ramp_ramp_{0}s_{1}_{2}_{3}s_{4}_{5}.bag'.format(throttle_duration / 50, throttle_min, throttle_max, brake_duration / 50, brake_min, brake_max)
    with rosbag.Bag(filename, 'w') as bag:
        now = 0
        now = lead_in(bag, now)
        now = ramp(bag, now, throttle_duration, throttle_min, throttle_max)
        now = ramp(bag, now, brake_duration, brake_min, brake_max)
        now = lead_out(bag, now)

def main():
    # Durations are in 1/50 of a second "ticks".

    # Throttle impulses.
    print('Generating throttle impulses...')
    throttle_duration_range = (50, 200, 50)
    throttle_value_range = (1550, 1800, 50)
    for duration in range(*throttle_duration_range):
        for value in range(*throttle_value_range):
            throttle_impulse(duration, value)

    # Throttle ramps.
    print('Generating throttle ramps...')
    throttle_duration_range = (50, 200, 50)
    throttle_max_range = (1550, 1800, 50)
    for duration in range(*throttle_duration_range):
        for max_value in range(*throttle_max_range):
            throttle_ramp(duration, 1500, max_value)

    # Brake impulses.
#    print('Generating brake impulses...')
#    throttle_duration_range = (50, 200, 50)
#    throttle_value_range = (1550, 1800, 50)
#    brake_duration_range = (50, 200, 50)
#    brake_value_range = (1450, 1200, -50)
#    for throttle_duration in range(*throttle_duration_range):
#        for throttle_value in range(*throttle_value_range):
#            for brake_duration in range(*brake_duration_range):
#                for brake_value in range(*brake_value_range):
#                    brake_impulse(throttle_duration, throttle_value, brake_duration, brake_value)

    # Brake ramps.
#    print('Generating brake ramps...')
#    throttle_duration_range = (50, 200, 50)
#    throttle_value_range = (1550, 1800, 50)
#    brake_duration_range = (50, 200, 50)
#    brake_min_range = (1450, 1200, -50)
#    brake_max_range = (1450, 1200, -50)
#    for throttle_duration in range(*throttle_duration_range):
#        for throttle_value in range(*throttle_value_range):
#            for brake_duration in range(*brake_duration_range):
#                for brake_min in range(*brake_min_range):
#                    brake_ramp(throttle_duration, throttle_value, brake_duration, brake_min, 1500)
#                for brake_max in range(*brake_max_range):
#                    brake_ramp(throttle_duration, throttle_value, brake_duration, 1500, brake_max)

    # Ramp ramps.
#    print('Generating ramp ramps...')
#    throttle_duration_range = (50, 200, 50)
#    throttle_max_range = (1550, 1800, 50)
#    throttle_min_range = (1550, 1800, 50)
#    brake_duration_range = (50, 200, 50)
#    brake_min_range = (1450, 1200, -50)
#    brake_max_range = (1450, 1200, -50)
#    for throttle_duration in range(*throttle_duration_range):
#        for throttle_max in range(*throttle_max_range):
#            for brake_duration in range(*brake_duration_range):
#                for brake_min in range(*brake_min_range):
#                    ramp_ramp(throttle_duration, 1500, throttle_max, brake_duration, brake_min, 1500)
#                for brake_max in range(*brake_max_range):
#                    ramp_ramp(throttle_duration, 1500, throttle_max, brake_duration, 1500, brake_max)
#        for throttle_min in range(*throttle_min_range):
#            for brake_duration in range(*brake_duration_range):
#                for brake_min in range(*brake_min_range):
#                    ramp_ramp(throttle_duration, throttle_min, 1500, brake_duration, brake_min, 1500)
#                for brake_max in range(*brake_max_range):
#                    ramp_ramp(throttle_duration, throttle_min, 1500, brake_duration, 1500, brake_max)

if __name__ == '__main__':
    main()
