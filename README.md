joy_feedback_ros
================

Create a simple ros node to allow setting rumble force feedback on a Xbox wired controller (and perhaps logitech F510 and anything else that is simlar) using fftest.c as a starting point.  Later try to integrate this into the proper https://github.com/ros-drivers/joystick_drivers repository.


    $ lsusb
    Bus 002 Device 003: ID 045e:028e Microsoft Corp. Xbox360 Controller


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
