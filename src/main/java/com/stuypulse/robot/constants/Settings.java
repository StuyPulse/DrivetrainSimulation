/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import java.nio.file.Path;

import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public final class Settings {
	
    SmartBoolean DEBUG_MODE = new SmartBoolean("Debug Mode", false);
	public static Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();

    public static void reportWarning(String message) {
        // DriverStation.reportWarning(message, false);
        System.out.println(message);
    }

	public interface Field {
		double FIELD_WIDTH = 27;
		double FIELD_HEIGHT = 54;
	}

	public interface Motion {
		double MAX_VELOCITY = 2.0;
        double MAX_ACCELERATION = 3.0;

		double TRACK_WIDTH = Units.inchesToMeters(26.9);
		// How accurate we think our model is
		double STATE_STDEV_LEFT = 3.0;
		double STATE_STDEV_RIGHT = 3.0;

		// How accurate we think our encoder/gyro data is
		double MEASURE_STDEV_LEFT = 0.01;
		double MEASURE_STDEV_RIGHT = 0.01;

		// qelms. State error tolerance. Decrease this to more heavily penalize state excursion, or make the controller behave more aggressively.
		double Q_LEFT = 8.0;
		double Q_RIGHT = 8.0;

		// relms. Control effort (voltage) tolerance. Decrease this to more heavily penalize control effort, or make the controller less aggressive. 12 is a good starting point because that is the (approximate) maximum voltage of a battery.
		double R_LEFT = 12.0;
		double R_RIGHT = 12.0;

		double MAX_VOLTAGE = 12.0;
		double DT = 0.020;

		double BASE_TURNING_SPEED = 0.5;

		double MAX_TELE_SPEED = Units.feetToMeters(17.3);

		DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

		public interface Encoders {

            public interface GearRatio {

                public interface Stages {
                    double INITIAL_STAGE = (11.0 / 50.0);

                    double HIGH_GEAR_STAGE = (50.0 / 34.0);
                    double LOW_GEAR_STAGE = (24.0 / 60.0);

                    double GRAYHILL_STAGE = (12.0 / 36.0);

                    double THIRD_STAGE = (34.0 / 50.0);

                    double EXTERNAL_STAGE = (1.0 / 1.0);
                }

                /** = 0.22666 */
                double GRAYHILL_TO_WHEEL =
                        Stages.GRAYHILL_STAGE * Stages.THIRD_STAGE * Stages.EXTERNAL_STAGE;
            }

            double WHEEL_DIAMETER = Units.inchesToMeters(4);
			double WHEEL_RADIUS = WHEEL_DIAMETER / 2.0;

            double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

            double GRAYHILL_PULSES_PER_REVOLUTION = 256;
            double GRAYHILL_DISTANCE_PER_PULSE =
                    (WHEEL_CIRCUMFERENCE / GRAYHILL_PULSES_PER_REVOLUTION)
                            * GearRatio.GRAYHILL_TO_WHEEL;

            boolean GRAYHILL_INVERTED = true;
        }
	}

	public interface SysID {
		double kV = 1.6658;
		double kA = 0.4515;
		double kVAngular = 3.0;
		double kAAngular = 1.0;
	}

	public interface PID {
		public interface DriveDistance {
			SmartNumber kP = new SmartNumber("PID/DriveDistance/kP", 0.3);
			SmartNumber kI = new SmartNumber("PID/DriveDistance/kI", 0.0);
			SmartNumber kD = new SmartNumber("PID/DriveDistance/kD", 0.0);
		}

		public interface DrivetrainAlign {
			SmartNumber kP = new SmartNumber("PID/DrivetrainAlign/kP", 0.1);
			SmartNumber kI = new SmartNumber("PID/DrivetrainAlign/kI", 0.0);
			SmartNumber kD = new SmartNumber("PID/DrivetrainAlign/kD", 0.0);
		}
		
	}

	public interface BangBang {
		SmartNumber SPEED = new SmartNumber("BangBang/Speed", 1.0);
	}
	
}
