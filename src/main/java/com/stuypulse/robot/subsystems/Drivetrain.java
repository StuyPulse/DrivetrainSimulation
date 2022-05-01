package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.NoMotor;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
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


public class Drivetrain extends SubsystemBase { 

    // Encoders 
    private final Encoder leftEncoder, rightEncoder;

    // Gyro
	private final AnalogGyro gyro;
	
	// Motors
    private final NoMotor[] leftMotors;
    private final NoMotor[] rightMotors;

	// Odometry
	private final DifferentialDriveOdometry odometry;
	private final Field2d field;

	// Drivetrain
	private final DifferentialDrive drivetrain;

	// Simulation
    private EncoderSim leftEncoderSim, rightEncoderSim;
    private AnalogGyroSim gyroSim;
	private DifferentialDrivetrainSim drivetrainSim;

	// State space control
	private final LinearSystemLoop<N2, N2, N2> loop;

    public Drivetrain() {
		// Add Motors
		leftMotors =
			new NoMotor[] {
				new NoMotor(),
				new NoMotor(),
				new NoMotor()
			};

		rightMotors =
			new NoMotor[] {
				new NoMotor(),
				new NoMotor(),
				new NoMotor()
			};
		
		// Create Differential Drive & Sim
		drivetrain =
			new DifferentialDrive(
				new MotorControllerGroup(leftMotors),
				new MotorControllerGroup(rightMotors));
        drivetrainSim = Settings.Drivetrain.System.MODEL;

		// Create Encoders
		leftEncoder = new Encoder(Ports.Grayhill.LEFT_A, Ports.Grayhill.LEFT_B); 
		rightEncoder = new Encoder(Ports.Grayhill.RIGHT_A, Ports.Grayhill.RIGHT_B);

		leftEncoder.setDistancePerPulse(Settings.Drivetrain.Encoders.GRAYHILL_DISTANCE_PER_PULSE);
		rightEncoder.setDistancePerPulse(Settings.Drivetrain.Encoders.GRAYHILL_DISTANCE_PER_PULSE);
		
		leftEncoder.reset();
		rightEncoder.reset();

		// Create Encoder Sims
		leftEncoderSim = new EncoderSim(leftEncoder);
		rightEncoderSim = new EncoderSim(rightEncoder);
		
        // Create Gyro & Sim
        gyro = new AnalogGyro(Ports.Gyro.CHANNEL);
        gyroSim = new AnalogGyroSim(gyro);

		// Create Odometry & Visualizer
		odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
		field = new Field2d();
        SmartDashboard.putData("Field", field);


        // state-space controller
        // A LQR uses feedback to create voltage commands.
		loop = Settings.Drivetrain.LOOPER;
	}

	public Field2d getField() {
		return field;
	}

	private double getLeftMotorSpeed() {
		double total = 0;

		for (NoMotor m : leftMotors) {
			total += m.get();
		}

		return total / leftMotors.length;
	}
	
    private double getRightMotorSpeed() {
		double total = 0;

		for (NoMotor m : rightMotors) {
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
		loop.setNextR(VecBuilder.fill(left * Settings.Drivetrain.Motion.MAX_TELEOP_SPEED, right * Settings.Drivetrain.Motion.MAX_TELEOP_SPEED));
		loop.correct(VecBuilder.fill(getLeftRate(), getRightRate()));
		loop.predict(Settings.dT);

		left = loop.getU(0);
		right = loop.getU(1);

		tankDrive(
            left / Settings.Drivetrain.MAX_VOLTAGE, 
            right / Settings.Drivetrain.MAX_VOLTAGE
        );
	}

	public void arcadeDrive(double speed, double rotation) {
		drivetrain.arcadeDrive(speed, rotation, false);
	}

	public void arcadeDriveKalman(double speed, double turn) {
		speed = SLMath.deadband(speed, 0.05);
		turn = SLMath.deadband(turn, 0.05);

		double left = speed + turn;
		double right = speed - turn;

		double mag = Math.max(1.0, Math.max(Math.abs(left), Math.abs(right)));
		left /= mag;
		right /= mag;

		tankDriveKalman(left, right);
	}

	public void curvatureDriveKalman(double xSpeed, double zRotation, double baseTS) {
        // Clamp all inputs to valid values
        xSpeed = SLMath.clamp(xSpeed, -1.0, 1.0);
        zRotation = SLMath.clamp(zRotation, -1.0, 1.0);
        baseTS = SLMath.clamp(baseTS, 0.0, 1.0);

        // Find the amount to slow down turning by.
        // This is proportional to the speed but has a base value
        // that it starts from (allows turning in place)
        double turnAdj = Math.max(baseTS, Math.abs(xSpeed));

        // Find the speeds of the left and right wheels
        double lSpeed = xSpeed + zRotation * turnAdj;
        double rSpeed = xSpeed - zRotation * turnAdj;

        // Find the maximum output of the wheels, so that if a wheel tries to go > 1.0
        // it will be scaled down proportionally with the other wheels.
        double scale = Math.max(1.0, Math.max(Math.abs(lSpeed), Math.abs(rSpeed)));

        lSpeed /= scale;
        rSpeed /= scale;

        // Feed the inputs to the drivetrain
        tankDriveKalman(lSpeed, rSpeed);
    }

    // Drives using curvature drive algorithm with automatic quick turn
    public void curvatureDriveKalman(double xSpeed, double zRotation) {
        curvatureDriveKalman(xSpeed, zRotation, Settings.Driver.BASE_TURN_SPEED);
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
		drivetrainSim.setPose(location);
		odometry.resetPosition(location, getGyroAngle().negative().getRotation2d());
	}

    public void reset() {
        reset(getPose());
    }


	// Periodic Functions

	@Override 
	public void simulationPeriodic() {
		drivetrainSim.setInputs(getLeftMotorSpeed() * RobotController.getInputVoltage(),
								getRightMotorSpeed() * RobotController.getInputVoltage());

		drivetrainSim.update(Settings.dT);

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
