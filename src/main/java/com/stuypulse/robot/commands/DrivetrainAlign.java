package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Camera;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * TODO:
 *  - use ifusers data (look into kallman filters for this)
 *  - add distance align
 *
 */

public class DrivetrainAlign extends CommandBase {
    
    private final Drivetrain drivetrain;
    private final Camera camera;

    private Controller angleController;
    private double targetAngle;

    public DrivetrainAlign(Drivetrain drivetrain, Camera camera) {
        this.drivetrain = drivetrain;
        this.camera = camera;

        angleController = new PIDController(Settings.PID.DrivetrainAlign.kP, Settings.PID.DrivetrainAlign.kI, Settings.PID.DrivetrainAlign.kD);

        targetAngle = 0.0;

        addRequirements(drivetrain);
    }

    public void initialize() {
        targetAngle = drivetrain.getRawGyroAngle() + camera.getXAngle().toDegrees();
    }


    public void execute() {
        drivetrain.arcadeDrive(
            0,
            angleController.update(targetAngle - drivetrain.getRawGyroAngle())
        );
    }

    public boolean isFinished() {
        return angleController.isDone(1.0);
    }


}
