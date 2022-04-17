package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings.BangBang;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.stuylib.control.BangBangController;
import com.stuypulse.stuylib.network.SmartNumber;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainDriveDistanceBangBang extends CommandBase {

	private BangBangController feedback;    
	private Drivetrain drivetrain;
	private final int distance;
	private SmartNumber error;

	public DrivetrainDriveDistanceBangBang(Drivetrain drivetrain, int distance) {
		this.drivetrain = drivetrain;
		this.distance = distance;

		this.feedback = new BangBangController(BangBang.SPEED.get());

		error = new SmartNumber("Control/Error", 0.0);

		addRequirements(drivetrain);
	}

	@Override
	public void execute() {
		error.set(distance - drivetrain.getDistance());
		
		double motorOutput = feedback.update(error.get());

		drivetrain.arcadeDrive(motorOutput, 0.0);
	}

	@Override
	public boolean isFinished() {
		return error.get() == 0;
	}

	@Override
	public void end(boolean interrupted){
		drivetrain.stop();
	}

}
