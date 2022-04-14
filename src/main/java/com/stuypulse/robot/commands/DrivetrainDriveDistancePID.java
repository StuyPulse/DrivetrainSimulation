package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings.PID;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.stuylib.control.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainDriveDistancePID extends CommandBase {

	private PIDController feedback;    
	private Drivetrain drivetrain;
	private final int distance;
	private double error;

	public DrivetrainDriveDistancePID(Drivetrain drivetrain, int distance) {
		this.drivetrain = drivetrain;
		this.distance = distance;

		this.feedback = new PIDController(PID.kP,PID.kI,PID.kD);

		addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		error = distance - drivetrain.getDistance();
		
		double motorOutput = feedback.update(error);

		drivetrain.arcadeDrive(motorOutput, 0.0);
	}

	@Override
	public boolean isFinished() {
		return error == 0;
	}

	@Override
	public void end(boolean interrupted){
		drivetrain.stop();
	}

}
