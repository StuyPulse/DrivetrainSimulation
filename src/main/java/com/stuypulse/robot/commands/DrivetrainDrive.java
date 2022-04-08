package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainDrive extends CommandBase {
    private Drivetrain drivetrain;

	public DrivetrainDrive(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);
	}

	public void execute() {
		// h
	}

	public boolean isFinished() {
		return false;
	}
}
