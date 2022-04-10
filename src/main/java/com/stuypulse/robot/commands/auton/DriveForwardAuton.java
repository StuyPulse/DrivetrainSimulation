package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.DrivetrainDriveDistancePID;
import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveForwardAuton extends SequentialCommandGroup {
    
	private Drivetrain drivetrain;

	public DriveForwardAuton(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		addRequirements(drivetrain);

		addCommands(
			new DrivetrainDriveDistancePID(drivetrain, 10)
		);
	}

}
