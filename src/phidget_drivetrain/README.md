


Worm gear front left and right
- pwm and dir pins to control them


- front_left_right_worm.py move
- front_left_right_worm.ino




ros flow
physical joystick -> ros joy -> python worm (manipulate) -> pwm and dir values -> worm gear front left and right

worm gear back left and right
physical joystick -> ros joy -> python worm (manipulate) -> servo values (0-180) -> worm gear back left and right

challenge
- back left and right (only 10amp motor driver so we need to slow down the turning )



pinout
