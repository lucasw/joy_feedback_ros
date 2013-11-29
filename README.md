joy_feedback_ros
================

Create a simple ros node to allow setting rumble force feedback on a Xbox wired controller (and perhaps logitech F510 and anything else that is simlar) using fftest.c as a starting point.  Later try to integrate this into the proper https://github.com/ros-drivers/joystick_drivers repository.

Current usage
=============

Set a strong and weak magnitude to 18000 and then play the (hard-coded) index 0

    roslaunch joy_feedback_ros joy_feedback.launch 
    rostopic pub -1 /rumble joy_feedback_ros/Rumble 18000 18000
    rostopic pub /play std_msgs/UInt16 1


A sine effect:

    rostopic pub /periodic joy_feedback_ros/Periodic  '{waveform: 2, period: 100, magnitude: 12000, offset: 0, phase: 0, envelope:  {attack_length: 200, attack_level: 0, fade_length: 200, fade_level: 0}}'
    rostopic pub /play std_msgs/UInt16 1


SAW_UP and SAW_DOWN don't work on the Xbox 360 controller, it produces an 'Invalid argument' from ioctl.

Useful information
==================

https://www.kernel.org/doc/Documentation/input/ff.txt

    - FF_CONSTANT can render constant force effects
    - FF_PERIODIC can render periodic effects with the following waveforms:
      - FF_SQUARE   square waveform
      - FF_TRIANGLE   triangle waveform
      - FF_SINE   sine waveform
      - FF_SAW_UP   sawtooth up waveform
      - FF_SAW_DOWN   sawtooth down waveform
      - FF_CUSTOM   custom waveform
    - FF_RAMP       can render ramp effects
    - FF_SPRING can simulate the presence of a spring
    - FF_FRICTION can simulate friction
    - FF_DAMPER can simulate damper effects
    - FF_RUMBLE rumble effects
    - FF_INERTIA    can simulate inertia
    - FF_GAIN gain is adjustable
    - FF_AUTOCENTER autocenter is adjustable

    Note: In most cases you should use FF_PERIODIC instead of FF_RUMBLE. All
      devices that support FF_RUMBLE support FF_PERIODIC (square, triangle,
      sine) and the other way around.

My current forcefeedback device:

    $ lsusb
    Bus 002 Device 003: ID 045e:028e Microsoft Corp. Xbox360 Controller


Test it with fftest:

https://github.com/flosse/linuxconsole/blob/master/utils/fftest.c

The rumble effects worked:

    $ fftest /dev/input/event13
    Force feedback test program.
    HOLD FIRMLY YOUR WHEEL OR JOYSTICK TO PREVENT DAMAGES

    Device /dev/input/event13 opened
    Axes query: 
    Effects: Periodic Rumble 
    Number of simultaneous effects: 16
    Upload effects[1]: Invalid argument
    Upload effects[2]: Invalid argument
    Upload effects[3]: Invalid argument
    Enter effect number, -1 to exit
    4
    Now Playing: Strong Rumble
    Enter effect number, -1 to exit
    5
    Now Playing: Weak Rumble

None of these effects worked:

    1
    Now Playing: Constant Force
    Enter effect number, -1 to exit
    2
    Now Playing: Spring Condition
    Enter effect number, -1 to exit
    3
    Now Playing: Damping Condition
    Enter effect number, -1 to exit
