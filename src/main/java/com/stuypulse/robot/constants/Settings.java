/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot.constants;

import java.nio.file.Path;

import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.network.SmartBoolean;
import com.stuypulse.stuylib.network.SmartNumber;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

/*-
 * File containing tunable settings for every subsystem on the robot.
 *
 * We use StuyLib's SmartNumber / SmartBoolean in order to have tunable
 * values that we can edit on Shuffleboard.
 */
public interface Settings {
	
    SmartBoolean DEBUG_MODE = new SmartBoolean("Debug Mode", false);
	Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();

    public static void reportWarning(String message) {
        // DriverStation.reportWarning(message, false);
        System.out.println(message);
    }

    double dT = 0.020;

    public interface Driver {
        double BASE_TURN_SPEED = 0.5;
    }

    public interface Drivetrain {
        double MAX_VOLTAGE = 12.0;
        double TRACK_WIDTH = Units.inchesToMeters(26.9);

        public interface Motion {
            double MAX_TELEOP_SPEED = Units.feetToMeters(17.3);
            double MAX_VELOCITY = 2.0;
            double MAX_ACCELERATION = 3.0;
            DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);
        }

        public interface Feedforward {
            double kV = 1.6658;
			double kA = 0.4515;
			double kVAngular = 3.0;
			double kAAngular = 2.0;

            LinearSystem<N2, N2, N2> PLANT = 
                LinearSystemId.identifyDrivetrainSystem(kV, kA, kVAngular, kAAngular);
        }

        public interface Kalman {
            // The observer fuses our encoder/gyro data and voltage inputs to reject noise.

            // How accurate we think our model is
            double STATE_STDEV_LEFT = 3.0;
            double STATE_STDEV_RIGHT = 3.0;

            // How accurate we think our encoder/gyro data is
            double MEASURE_STDEV_LEFT = 0.01;
            double MEASURE_STDEV_RIGHT = 0.01;

            KalmanFilter<N2, N2, N2> FILTER =
                new KalmanFilter<>(
                    Nat.N2(),
                    Nat.N2(),
                    Feedforward.PLANT,
                    VecBuilder.fill(STATE_STDEV_LEFT, STATE_STDEV_RIGHT), 
                    VecBuilder.fill(MEASURE_STDEV_LEFT, MEASURE_STDEV_RIGHT), 
                    dT
                );

        }

        public interface LQR {
            // The observer fuses our encoder/gyro data and voltage inputs to reject noise.

            // qelms. State error tolerance. Decrease this to more heavily penalize state excursion, or make the controller behave more aggressively.
            double Q_LEFT = 8.0;
            double Q_RIGHT = 8.0;

            // relms. Control effort (voltage) tolerance. Decrease this to more heavily penalize control effort, or make the controller less aggressive. 12 is a good starting point because that is the (approximate) maximum voltage of a battery.
            double R_LEFT = 12.0;
            double R_RIGHT = 12.0;

            LinearQuadraticRegulator<N2, N2, N2> CONTROLLER = 
                new LinearQuadraticRegulator<>(
                    Feedforward.PLANT,
                    VecBuilder.fill(Q_LEFT, Q_RIGHT), 
                    VecBuilder.fill(R_LEFT, R_RIGHT), 
                    dT
                );
        }

         // Loop that brings it all together
        LinearSystemLoop<N2, N2, N2> LOOPER =
            new LinearSystemLoop<>(Feedforward.PLANT, LQR.CONTROLLER, Kalman.FILTER, MAX_VOLTAGE, dT);
        
        public interface System {

            int MOTORS = 3;

            public interface Noise {
                // ?? 
            }

            DifferentialDrivetrainSim MODEL = 
                new DifferentialDrivetrainSim(
                    Feedforward.PLANT,
                    DCMotor.getNeo550(MOTORS),
                    Encoders.GearRatio.Stages.HIGH_GEAR_STAGE, // TODO: def not correct?       
                    TRACK_WIDTH,                  
                    Encoders.WHEEL_RADIUS,
        
                    // Measurement noise deviation
                    // These should be set, and then the kalman filters should be tuned assuming these noise
                    // values are not known
                    VecBuilder.fill(
                        0.001, 0.001, // x/y
                        0.001,        // heading
                        0.01, 0.01,   // l/r velocity
                        0.005, 0.005  // l/r position
                    )
                );
        }

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

				double TURRET_GEARING = 1.0;
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

    public interface Turret {

        double TURRET_DISTANCE_PER_PULSE = 1.0 / 24.0;

        public interface System {
            double kV = 0.5;
            double kA = 0.1; 

            int MOTORS = 1;
            double GEARING = 1.0;
            
            public static FlywheelSim getSystem() {
                return new FlywheelSim(
                    LinearSystemId.identifyVelocitySystem(kV, kA),
                    DCMotor.getNeo550(MOTORS),
                    GEARING
                );
            }
        }

        public interface Turning {
            double MAX_ANGLE = +190.0;
            double MIN_ANGLE = -190.0;

            SmartNumber kP = new SmartNumber("Turret/kP", 2.5);
			SmartNumber kI = new SmartNumber("Turret/kI", 0.2);
			SmartNumber kD = new SmartNumber("Turret/kD", 0.1);

            public static PIDController getController() {
                return new PIDController(kP, kI, kD);
            }
        }

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
