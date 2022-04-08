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

    // pov computer plate
	private final AnalogGyro gyro;
	
	// Motors
    private final CANSparkMax[] leftMotors;
    private final CANSparkMax[] rightMotors;

	// Odometry kk
	private final DifferentialDriveOdometry odometry;
	private final Field2d field;

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
				new CANSparkMax(Ports.Drivetrain.LEFT_TOP, MotorType.kBrushless)
			};

		rightMotors =
			new CANSparkMax[] {
				new CANSparkMax(-1, MotorType.kBrushless),
				new CANSparkMax(-1, MotorType.kBrushless),
				new CANSparkMax(-1, MotorType.kBrushless)
			};

        // Create Drivetrain Sim (skull emoji)
        drivetrainSim = new DifferentialDrivetrainSim(
			LinearSystemId.identifyDrivetrainSystem(Settings.SysID.kV, Settings.SysID.kA, Settings.SysID.kVAngular, Settings.SysID.kAAngular),
			DCMotor.getNEO(3),       // 2 NEO motors on each side of the drivetrain.
			7.29,                    // gear ratio
			0.7112,                  // tradk width
			Units.inchesToMeters(3), // wheel radius

			// measurement noise deviation ???
			VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
		
		// Create Encoders
		leftEncoder = new Encoder(-1, -1); 
		rightEncoder = new Encoder(-1, -1); // oh in dorcas theyre port constants

		// Create Encoder Sims
		leftEncoderSim = new EncoderSim(leftEncoder);
		rightEncoderSim = new EncoderSim(rightEncoder);

        // Create Gyro
        gyro = new AnalogGyro(-1);

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
