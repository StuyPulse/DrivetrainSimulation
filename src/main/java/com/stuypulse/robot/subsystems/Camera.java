package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * TODO:
 *  - add a "DelayFilter" for camera data
 *  - take x-angle from a point on the rim of the hub (cast a ray into a circle)
 *
 */

public class Camera extends SubsystemBase {
    
    private final Drivetrain drivetrain;
    private Vector2D hub;

    public Camera(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // TODO: better way to find center? 
        this.hub = new Vector2D(
            Units.feetToMeters(54.0) / 2.0,
            Units.feetToMeters(27.0) / 2.0
        );

        addGoal(drivetrain.getField());
    }

    /** FIELD **/

    private void addGoal(Field2d field) {
        field.getObject("goal").setPose(new Pose2d(hub.x, hub.y, Rotation2d.fromDegrees(0)));
    }

    /** SIMULATED CALCULATIONS */

    private Angle getAngleToHub() {
        Pose2d pose = drivetrain.getPose();
        Vector2D pos = new Vector2D(pose.getX(), pose.getY());
        Angle ang = Angle.fromDegrees(pose.getRotation().getDegrees());

        return hub.sub(pos).getAngle().sub(ang);
    }

    private double getRawDistance() {
        Pose2d pose = drivetrain.getPose();
        Vector2D pos = new Vector2D(pose.getX(), pose.getY());

        return hub.sub(pos).magnitude();
    }

    /** PUBLIC API **/

    public boolean hasTarget() {
        return Math.abs(getAngleToHub().toDegrees()) < 27.0 && 
               getRawDistance() < Units.feetToMeters(30);
    }

    public Angle getXAngle() {
        if (!hasTarget()) {
            Settings.reportWarning("Unable to find Limelight target [getXAngle()]");
        }

        return getAngleToHub().negative();
    }

    public double getDistance() {
        if (!hasTarget()) {
            Settings.reportWarning("Unable to find Limelight target [getDistance()]");
        }

        return getRawDistance() - Units.feetToMeters(2);
    }

    /*
    public Angle getYAngle() {
        if (!hasTarget()) {
            Settings.reportWarning("Unable to find Limelight target [getYAngle()]");
        }

        technically possible to work "backwards" using the distance and hub height 
        to find the angle to the horizontal and then subtract the horizontal angle of 
        the limelight 

        atan2(hub height, distance) - limelight_pitch
    }
    */

}
