package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DrivetrainReset extends InstantCommand {

	private Drivetrain drivetrain;

	public DrivetrainReset(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
	}

	@Override
	public void initialize() {
		drivetrain.resetSensors();
	}

}
