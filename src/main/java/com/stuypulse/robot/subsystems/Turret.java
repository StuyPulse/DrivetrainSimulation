package com.stuypulse.robot.subsystems;


import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    
    // private Vector2D turretVector;
    private Drivetrain drivetrain;
    private FieldObject2d turret;



    public Turret (Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.turret = drivetrain.getField().getObject("turret");

        setPose(drivetrain, Rotation2d.fromDegrees(0.0));

    }

    private void setPose(Drivetrain drivetrain, Rotation2d angle) {

        turret.setPose(new Pose2d(drivetrain.getPose().getX(), drivetrain.getPose().getY(), angle));
    }

    public void setPose(Rotation2d angle) {
        setPose(drivetrain, angle);
    }

    public Rotation2d getAngle() {
        return turret.getPose().getRotation();
    }



    @Override
    public void periodic() {

        setPose(drivetrain, getAngle());
    }



}
