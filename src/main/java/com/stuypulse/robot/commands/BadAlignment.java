package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Camera;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 * This alignment uses only camera data to align to the goal.
 * 
 * It is not ideal because camera data is delayed. Therefore,
 * when the robot reacts to some limelight error and moves, the
 * limelight will report old data (as if the robot had not moved).
 * 
 * This can cause oscillations when alignment. 
 * 
 */
public class BadAlignment extends CommandBase {
    
    private final Drivetrain drivetrain;
    private final Controller angleController;
    private final Controller distanceController;

    private final Camera camera;

    public BadAlignment(Drivetrain drivetrain, Camera camera) {
        this.drivetrain = drivetrain;
        this.camera = camera;

        angleController = new PIDController(Settings.PID.DrivetrainAlign.kP, Settings.PID.DrivetrainAlign.kI, Settings.PID.DrivetrainAlign.kD);
        distanceController = new PIDController(0.1, 0.0, 0.02);

        addRequirements(drivetrain);
    }

    public void execute() {
        drivetrain.arcadeDrive(distanceController.update(camera.getDistance() - Units.inchesToMeters(150)), angleController.update(camera.getXAngle().toDegrees()));
    }



}
