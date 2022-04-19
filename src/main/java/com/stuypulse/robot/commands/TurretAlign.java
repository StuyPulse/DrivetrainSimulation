package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Ports.Drivetrain;
import com.stuypulse.robot.subsystems.Camera;
import com.stuypulse.robot.subsystems.Turret;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurretAlign extends CommandBase {

    private final Camera camera;
    private final Turret turret;
    private Controller angleController;
    private double targetAngle;

    public TurretAlign(Camera camera, Turret turret) {
        this.turret = turret;
        this.camera = camera;

        angleController = new PIDController(Settings.PID.DrivetrainAlign.kP, Settings.PID.DrivetrainAlign.kI, Settings.PID.DrivetrainAlign.kD);

        targetAngle = 0.0;

        addRequirements(turret);
    }

    public void initialize() {
        targetAngle = turret.getAngle().getDegrees() + camera.getXAngle().toDegrees();
    }

    public void execute() {
        
        turret.setPose(
            Rotation2d.fromDegrees(
                angleController.update(targetAngle - turret.getAngle().getDegrees())
            )
        );
    }

    public boolean isFinished() {
        return angleController.isDone(1.0);
    }


}