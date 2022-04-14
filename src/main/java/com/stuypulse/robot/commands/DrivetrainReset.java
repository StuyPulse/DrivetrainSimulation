package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.stuylib.control.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainReset extends CommandBase {

	private Drivetrain drivetrain;

	public DrivetrainReset(Drivetrain drivetrain) {
		this.drivetrain = drivetrain;
	}

	@Override
	public void execute() {
		drivetrain.resetSensors();
	}

}
