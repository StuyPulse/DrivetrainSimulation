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

	

	public interface StateSpace {
		// How accurate we think our model is
		double STATE_STDEV_LEFT = 3.0;
		double STATE_STDEV_RIGHT = 3.0;

		// How accurate we think our encoder/gyro data is
		double MEASURE_STDEV_LEFT = 0.01;
		double MEASURE_STDEV_RIGHT = 0.01;

		// qelms. State error tolerance. Decrease this to more heavily penalize state excursion, or make the controller behave more aggressiLEFTy.
		double Q_LEFT = 8.0;
		double Q_RIGHT = 8.0;

		// relms. Control effort (voltage) tolerance. Decrease this to more heavily penalize control effort, or make the controller less aggressive. 12 is a good starting point because that is the (approximate) maximum voltage of a battery.
		double R_LEFT = 12.0;
		double R_RIGHT = 12.0;

		double MAX_VOLTAGE = 12.0;
		double DT = 0.020;
	}

	public interface SysID {
		double kV = 1.98;
		double kA = 0.2;
		double kVAngular = 1.5;
		double kAAngular = 0.3;
	}

	public interface PID {
		SmartNumber kP = new SmartNumber("PID/kP", 0.1);
		SmartNumber kI = new SmartNumber("PID/kI", 0.0);
		SmartNumber kD = new SmartNumber("PID/kD", 0.0);
	}

	public interface BangBang {
		SmartNumber SPEED = new SmartNumber("BangBang/Speed", 1.0);
	}
	
}
