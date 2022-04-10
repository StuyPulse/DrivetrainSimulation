package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.stuylib.input.Gamepad;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainDriveDistancePID extends CommandBase {
    
	private Drivetrain drivetrain;
	private final int distance;

	public DrivetrainDriveDistancePID(Drivetrain drivetrain, int distance) {
		this.drivetrain = drivetrain;
		this.distance = distance;

		addRequirements(drivetrain);
	}

	@Override
	public void execute() {

	}

}
