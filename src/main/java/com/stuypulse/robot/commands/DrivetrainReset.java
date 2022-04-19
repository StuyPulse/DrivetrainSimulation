package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class DrivetrainReset extends InstantCommand {

	private Drivetrain drivetrain;

	public DrivetrainReset(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
	}

	@Override
	public void initialize() {
		drivetrain.reset(new Pose2d(0,0,Rotation2d.fromDegrees(0)));
	}

}
