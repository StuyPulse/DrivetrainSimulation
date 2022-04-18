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

	public DrivetrainDrive(Drivetrain drivetrain, Gamepad gamepad) {
		this.drivetrain = drivetrain;
		this.gamepad = gamepad;

		addRequirements(drivetrain);
	}

	@Override
	public void initialize() {
		drivetrain.resetSensors();
	}

	@Override
	public void execute() {
		drivetrain.arcadeDriveKalman(gamepad.getLeftY(), gamepad.getLeftX());
	}

}