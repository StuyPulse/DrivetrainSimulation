package com.stuypulse.robot.subsystems;


import com.stuypulse.robot.constants.Field;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    
    private Drivetrain drivetrain;
    private FieldObject2d turretSim;

    private Vector2D target;

    public Turret(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.turretSim = drivetrain.getField().getObject("turret");

        target = Field.HUB;
    }

    public void pointAt(Vector2D target) {
        this.target = target;
    }

    private void setPose(Angle angle) {
        Pose2d robotPose = drivetrain.getPose();
        turretSim.setPose(new Pose2d(robotPose.getTranslation(), Rotation2d.fromDegrees(angle.toDegrees())));
    }

    private Angle getTargetAngle() {

        Pose2d robotPose = drivetrain.getPose();
        Vector2D pos = new Vector2D(robotPose.getX(), robotPose.getY());
        Angle ang = Angle.fromDegrees(robotPose.getRotation().getDegrees()); // el negativo???

        if (target == null) {
            return ang;
        }

        Vector2D toTarget = target.sub(pos);
        return toTarget.getAngle();
    }

    @Override
    public void periodic() {
        setPose(getTargetAngle());
    }



}
