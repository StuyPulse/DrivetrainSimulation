package com.stuypulse.robot.subsystems;


import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    
    private Vector2D turretVector;
    private Drivetrain drivetrain;
    private FieldObject2d turret;



    public Turret (Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.turret = drivetrain.getField().getObject("turret");

        setPose(drivetrain);
    }

    private void setPose(Drivetrain drivetrain) {
        this.turretVector = new Vector2D(
            drivetrain.getPose().getX(),
            drivetrain.getPose().getY()
        );

        turret.setPose(new Pose2d(turretVector.x, turretVector.y, Rotation2d.fromDegrees(0)));
    }



    @Override
    public void periodic() {

        setPose(drivetrain);
    }



}
