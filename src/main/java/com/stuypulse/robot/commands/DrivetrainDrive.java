package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.math.SLMath;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainDrive extends CommandBase {
    
	private Drivetrain drivetrain;
	private Gamepad gamepad;

	private final LinearSystem<N2, N2, N2> drivetrainPlant; // The drivetrain model
  	private final KalmanFilter<N2, N2, N2> observer; // The observer fuses our encoder/gyro data and voltage inputs to reject noise.
	private final LinearQuadraticRegulator<N2, N2, N2> controller; // A LQR uses feedback to create voltage commands.
	private final LinearSystemLoop<N2, N2, N2> loop; // Loop that brings it all together

	public DrivetrainDrive(Drivetrain drivetrain, Gamepad gamepad) {
		this.drivetrain = drivetrain;
		this.gamepad = gamepad;

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
				VecBuilder.fill(Settings.StateSpace.STATE_STDEV_LEFT, Settings.StateSpace.STATE_STDEV_RIGHT), 
				VecBuilder.fill(Settings.StateSpace.MEASURE_STDEV_LEFT, Settings.StateSpace.MEASURE_STDEV_RIGHT), 
				Settings.StateSpace.DT
			);

		controller =
			new LinearQuadraticRegulator<>(
				drivetrainPlant,
				VecBuilder.fill(Settings.StateSpace.Q_LEFT, Settings.StateSpace.Q_RIGHT), 
				VecBuilder.fill(Settings.StateSpace.R_LEFT, Settings.StateSpace.R_RIGHT), 
				Settings.StateSpace.DT
			);
		
		loop =
			new LinearSystemLoop<>(
				drivetrainPlant, 
				controller, 
				observer, 
				Settings.StateSpace.MAX_VOLTAGE, 
				Settings.StateSpace.DT
			);

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		drivetrain.resetSensors();
	}

	@Override
	public void execute() {
		double speed = gamepad.getLeftY();
		double turn = gamepad.getLeftX();

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

		loop.setNextR(VecBuilder.fill(left, right));
		loop.correct(VecBuilder.fill(drivetrain.getLeftRate(), drivetrain.getRightRate()));
		loop.predict(Settings.StateSpace.DT);

		left = loop.getU(0);
		right = loop.getU(1);

		drivetrain.tankDrive(left / Settings.StateSpace.MAX_VOLTAGE, right / Settings.StateSpace.MAX_VOLTAGE);
	}

}