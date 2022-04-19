/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.util.Units;

public interface Field {
    double FIELD_WIDTH = Units.feetToMeters(54);
    double FIELD_HEIGHT = Units.feetToMeters(27);

    Vector2D HUB = new Vector2D(FIELD_WIDTH/2, FIELD_HEIGHT/2);
}
