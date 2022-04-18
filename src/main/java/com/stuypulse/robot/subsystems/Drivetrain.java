package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/*
h
*/

public class Drivetrain extends SubsystemBase { 

    // Encoders 
    private final Encoder leftEncoder, rightEncoder;

    // Gyro
	private final AnalogGyro gyro;
	
	// Motors
    private final CANSparkMax[] leftMotors;
    private final CANSparkMax[] rightMotors;

	// Odometry
	private final DifferentialDriveOdometry odometry;
	private final Field2d field;

	// Drivetrain
	private final DifferentialDrive drivetrain;

	// Simulation
    private EncoderSim leftEncoderSim, rightEncoderSim;
    private AnalogGyroSim gyroSim;
	private DifferentialDrivetrainSim drivetrainSim;

	// State space
	private final LinearSystem<N2, N2, N2> drivetrainPlant; // The drivetrain model
	private final KalmanFilter<N2, N2, N2> observer; // The observer fuses our encoder/gyro data and voltage inputs to reject noise.
	private final LinearQuadraticRegulator<N2, N2, N2> controller; // A LQR uses feedback to create voltage commands.
	private final LinearSystemLoop<N2, N2, N2> loop; // Loop that brings it all together

    public Drivetrain() {
		// Add Motors
		leftMotors =
			new CANSparkMax[] {
				new CANSparkMax(Ports.Drivetrain.LEFT_TOP, MotorType.kBrushless),
				new CANSparkMax(Ports.Drivetrain.LEFT_MIDDLE, MotorType.kBrushless),
				new CANSparkMax(Ports.Drivetrain.LEFT_BOTTOM, MotorType.kBrushless)
			};

		rightMotors =
			new CANSparkMax[] {
				new CANSparkMax(Ports.Drivetrain.RIGHT_TOP, MotorType.kBrushless),
				new CANSparkMax(Ports.Drivetrain.RIGHT_MIDDLE, MotorType.kBrushless),
				new CANSparkMax(Ports.Drivetrain.RIGHT_BOTTOM, MotorType.kBrushless)
			};
		
		// Create Differential Drive
		drivetrain =
			new DifferentialDrive(
				new MotorControllerGroup(leftMotors),
				new MotorControllerGroup(rightMotors));
		
				
        // Create Drivetrain Sim
        drivetrainSim = new DifferentialDrivetrainSim(
			LinearSystemId.identifyDrivetrainSystem(Settings.SysID.kV, Settings.SysID.kA, Settings.SysID.kVAngular, Settings.SysID.kAAngular),
			DCMotor.getNEO(3),
			Settings.Motion.Encoders.GearRatio.Stages.HIGH_GEAR_STAGE,           
			Settings.Motion.TRACK_WIDTH,                  
			Settings.Motion.Encoders.WHEEL_RADIUS,

			// Measurement noise deviation
			VecBuilder.fill(
				0.001, 0.001, // x/y
				0.001,        // heading
				Settings.Motion.MEASURE_STDEV_LEFT, Settings.Motion.MEASURE_STDEV_RIGHT,     // l/r velocity
				0.005, 0.005  // l/r position
			)
		);
		
		// Create Encoders
		leftEncoder = new Encoder(Ports.Grayhill.LEFT_A, Ports.Grayhill.LEFT_B); 
		rightEncoder = new Encoder(Ports.Grayhill.RIGHT_A, Ports.Grayhill.RIGHT_B);

		// Create Encoder Sims
		leftEncoderSim = new EncoderSim(leftEncoder);
		rightEncoderSim = new EncoderSim(rightEncoder);
		
		leftEncoder.setDistancePerPulse(Settings.Motion.Encoders.GRAYHILL_DISTANCE_PER_PULSE);
		rightEncoder.setDistancePerPulse(Settings.Motion.Encoders.GRAYHILL_DISTANCE_PER_PULSE);
		
		leftEncoder.reset();
		rightEncoder.reset();

        // Create Gyro
        gyro = new AnalogGyro(Ports.Gyro.CHANNEL);

        // Create Gyro Sim
        gyroSim = new AnalogGyroSim(gyro);

		// Create Odometry
		odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
		field = new Field2d();

		
		drivetrainPlant =
			LinearSystemId.identifyDrivetrainSystem(
				Settings.SysID.kV, 
				Settings.SysID.kA, 
				Settings.SysID.kVAngular, 
				Settings.SysID.kAAngular
			);

		observer =
			new KalmanFilter<>(
				Nat.N2(),
				Nat.N2(),
				drivetrainPlant,
				VecBuilder.fill(Settings.Motion.STATE_STDEV_LEFT, Settings.Motion.STATE_STDEV_RIGHT), 
				VecBuilder.fill(Settings.Motion.MEASURE_STDEV_LEFT, Settings.Motion.MEASURE_STDEV_RIGHT), 
				Settings.Motion.DT
			);

		controller =
			new LinearQuadraticRegulator<>(
				drivetrainPlant,
				VecBuilder.fill(Settings.Motion.Q_LEFT, Settings.Motion.Q_RIGHT), 
				VecBuilder.fill(Settings.Motion.R_LEFT, Settings.Motion.R_RIGHT), 
				Settings.Motion.DT
			);
		
		loop =
			new LinearSystemLoop<>(
				drivetrainPlant, 
				controller, 
				observer, 
				Settings.Motion.MAX_VOLTAGE, 
				Settings.Motion.DT
			);

        // put data on SmartDashboard
        SmartDashboard.putData("Field", field);

	}
	

	private double getLeftMotorSpeed() {
		double total = 0;

		for (CANSparkMax m : leftMotors) {
			total += m.get();
		}

		return total / leftMotors.length;
	}
	
    private double getRightMotorSpeed() {
		double total = 0;

		for (CANSparkMax m : rightMotors) {
            total += m.get();
		}
        
        return total / rightMotors.length;
	}

    private void updateOdometry() {
        odometry.update(gyro.getRotation2d(),
                    leftEncoder.getDistance(),
                    rightEncoder.getDistance());
    }

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	// Driving Commands

	public void stop() {
		drivetrain.stopMotor();
	}

	public void tankDrive(double left, double right) {
		drivetrain.tankDrive(left, right, false);
	}

	public void tankDriveKalman(double left, double right) {
		loop.setNextR(VecBuilder.fill(left, right));
		loop.correct(VecBuilder.fill(getLeftRate(), getRightRate()));
		loop.predict(Settings.Motion.DT);

		left = loop.getU(0);
		right = loop.getU(1);

		drivetrain.tankDrive(left / Settings.Motion.MAX_VOLTAGE, right / Settings.Motion.MAX_VOLTAGE);
	}

	public void arcadeDrive(double speed, double rotation) {
		drivetrain.arcadeDrive(speed, rotation, false);
	}

	public void arcadeDriveKalman(double speed, double turn) {
		speed = SLMath.deadband(speed, 0.05);
		turn = SLMath.deadband(turn, 0.05);

		// Manual arcade drive implementation WOOZY
		double biggerInput = 
			Math.signum(speed) * 
			Math.max(
				Math.abs(speed),
				Math.abs(turn)
			);

		double left, right;

		if (speed >= 0.0) {
			if (turn >= 0.0) {
				left = biggerInput;
				right = speed - turn;		
			} else {
				left = speed + turn;
				right = biggerInput;
			}
		} else {
			if (turn >= 0) {
				left = speed + turn;
				right = biggerInput;
			} else {
				left = biggerInput;
				right = speed - turn;
			}
		}

		double mag = Math.max(Math.abs(left), Math.abs(right));
		if (mag > 1.0) {
			left /= mag;
			right /= mag;
		}

		tankDriveKalman(left, right);
	}

	// Encoder functions
	public double getRightDistance() {
		return rightEncoder.getDistance();
	}

	public double getLeftDistance() {
		return leftEncoder.getDistance();
	}

	public double getDistance() {
		return  (getLeftDistance() + getRightDistance() ) / 2;
	}

	public double getRightRate() {
		return rightEncoder.getRate();
	}

	public double getLeftRate() {
		return leftEncoder.getRate();
	}

	public double getRate() {
		return (getLeftRate() + getRightRate()) / 2;
	}

	public void resetRightEncoder() {
		rightEncoder.reset();
	}

	public void resetLeftEncoder() {
		leftEncoder.reset();
	}

	// Gyro Functions
	public double getRawGyroAngle() {
		return gyro.getAngle();
	}

	public Angle getGyroAngle() {
		return Angle.fromDegrees(getRawGyroAngle());
	}

	public void resetGyro() {
		gyro.reset();	
	}

	// Reset sensors
	public void resetSensors() {
		resetLeftEncoder();
		resetRightEncoder();
		resetGyro();
	}

	public void reset(Pose2d location) {
		resetSensors();
		odometry.resetPosition(location, getGyroAngle().negative().getRotation2d());
	}


	// Periodic Functions

	@Override 
	public void simulationPeriodic() {
		drivetrainSim.setInputs(getLeftMotorSpeed() * RobotController.getInputVoltage(),
								getRightMotorSpeed() * RobotController.getInputVoltage());

		drivetrainSim.update(Settings.Motion.DT);

		// update sensors
		leftEncoderSim.setDistance(drivetrainSim.getLeftPositionMeters());
		leftEncoderSim.setRate(drivetrainSim.getLeftVelocityMetersPerSecond());
		
		rightEncoderSim.setDistance(drivetrainSim.getRightPositionMeters());
		rightEncoderSim.setRate(drivetrainSim.getRightVelocityMetersPerSecond());
		
		gyroSim.setAngle(-drivetrainSim.getHeading().getDegrees());
	}

    @Override
    public void periodic() {
        updateOdometry();
        field.setRobotPose(odometry.getPoseMeters());
	}
}
