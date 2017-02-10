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
    while now < start + 100:
        if now >= next_auto:
            if now >= start + 50:
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

def throttle_impulse(bag, now, duration, value):
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

def impulse_test(filename, duration, value):
    with rosbag.Bag(filename, 'w') as bag:
        now = 0
        now = lead_in(bag, now)
        now = throttle_impulse(bag, now, duration, value)
        now = lead_out(bag, now)

def main():
    impulse_test('impulse_3.0s_1600.bag', 150, 1600)

if __name__ == '__main__':
    main()
