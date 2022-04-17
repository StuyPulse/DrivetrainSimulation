/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public final class Settings {
	
    SmartBoolean DEBUG_MODE = new SmartBoolean("Debug Mode", false);

	public interface SysID {
		double kV = 1.98;
		double kA = 0.2;
		double kVAngular = 1.5;
		double kAAngular = 0.3;	
	}

	public interface PID {
		SmartNumber kP = new SmartNumber("PID/kP", 1.0);
		SmartNumber kI = new SmartNumber("PID/kI", 1.0);
		SmartNumber kD = new SmartNumber("PID/kD", 0.0);
	}

	public interface BangBang {
		SmartNumber SPEED = new SmartNumber("BangBang/Speed", 1.0);
	}
	
}
