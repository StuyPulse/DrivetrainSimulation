/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

/** This file contains the different ports of motors, solenoids and sensors */
public final class Ports {
    public interface Gamepad {
        int DRIVER = 0;
        int OPERATOR = 1;
        int DEBUGGER = 2;
    }
    public interface Drivetrain {
        int LEFT_TOP = -1;
        int LEFT_MIDDLE = -1;
        int LEFT_BOTTOM = -1;
        int RIGHT_TOP = -1;
        int RIGHT_MIDDLE = -1;
        int RIGHT_BOTTOM = -1;
    }
}
