package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.commands.DrivetrainDriveDistanceBangBang;
import com.stuypulse.robot.commands.DrivetrainDriveDistancePID;
import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveForwardAuton extends SequentialCommandGroup {
    
	private Drivetrain drivetrain;
	private SendableChooser<Command> auto;

	// maybe make this setting?
	private static final int DIST = 10;

	public DriveForwardAuton(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;

		auto = new SendableChooser<Command>();

		auto.setDefaultOption("PID", new DrivetrainDriveDistancePID(drivetrain, DIST));
		auto.addOption("Bang Bang", new DrivetrainDriveDistanceBangBang(drivetrain, DIST));

		SmartDashboard.putData("Control/Controller", auto);

		addRequirements(drivetrain);

		addCommands(
			auto.getSelected()
		);
	}

}
