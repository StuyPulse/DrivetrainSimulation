package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.stuylib.input.Gamepad;
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
		drivetrain.curvatureDrive(gamepad.getLeftY(), gamepad.getLeftX());
	}

}