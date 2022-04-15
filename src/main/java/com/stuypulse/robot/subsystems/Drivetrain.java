package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
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
			DCMotor.getNEO(3),       // 2 NEO motors on each side of the drivetrain.
			7.29,                    // gear ratio
			0.7112,                  // tradk width
			Units.inchesToMeters(3), // wheel radius

			// measurement noise deviation ???
			VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
		
		// Create Encoders
		leftEncoder = new Encoder(Ports.Grayhill.LEFT_A, Ports.Grayhill.LEFT_B); 
		rightEncoder = new Encoder(Ports.Grayhill.RIGHT_A, Ports.Grayhill.RIGHT_B);

		// Create Encoder Sims
		leftEncoderSim = new EncoderSim(leftEncoder);
		rightEncoderSim = new EncoderSim(rightEncoder);

        // Create Gyro
        gyro = new AnalogGyro(Ports.Gyro.CHANNEL);

        // Create Gyro Sim
        gyroSim = new AnalogGyroSim(gyro);

		// Create Odometry
		odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
		field = new Field2d();

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

	// Driving Commands

	public void stop() {
		drivetrain.stopMotor();
	}

	public void tankDrive(double left, double right) {
		drivetrain.tankDrive(left, right, false);
	}

	public void arcadeDrive(double speed, double rotation) {
		drivetrain.arcadeDrive(speed, rotation, false);
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

	public void resetRightEncoder() {
		rightEncoder.reset();
	}

	public void resetLeftEncoder() {
		leftEncoder.reset();
	}

	// Gyro Functions
	public double getGyroAngle() {
		return gyro.getAngle();
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


	// Periodic Functions

	@Override 
	public void simulationPeriodic() {
		drivetrainSim.setInputs(getLeftMotorSpeed() * RobotController.getInputVoltage(),
								getRightMotorSpeed() * RobotController.getInputVoltage());

		// TODO: replace with a timer
		drivetrainSim.update(0.02);

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
