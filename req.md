Constraints:
    - motors are non-encoded , its a dc motor with sabertooth motor driver , connected to D18 pin of the esp32 
    - 

Rules:
    1. when one gps_coordinate is received from telemetry -> target_coordinate, do:
    i gave this as a example to you but if you have any better idea you can implement it properly as a professional robotics engineer as it a very important and sensitive project . it will collect base and rover both  gps data then calculate bearing and heading error and rotate the motor to the correct direction.It should calculate the bearing move the motor left/right according to this and keeps scaning and doing the task simultaneously. 

       Example :  scan_GPS() -> current_coordinate, scan_Compass() -> current_heading, calc_bearing() -> bearing
        while abs(current_heading-bearing > threshold_angle) {
            rotate_motor_left/right()
            scan_Compass()
        }
        stop_motor()

        if you have any better idea , you can implement it properly as a professional robotics engineer.

    2. when the system is on, consider the motors position at 180 degree., when ever it turns on , it will consider its current position as 180 degree 

    3. it can rotate to min=0, max=360 degree

    4. assume first i move the motor from 0 to 350 , now i want to move it to 10 degree again , 
    
    if it need to shift from 350 to 10, it takes the longer path so that the wires does not tangle up ,, because it's max is 360 so it cant just directly jump to 10 crossing the 360 max range , it will follow the same path to move to 10 which it used to come to 350 from 0 at first ,  