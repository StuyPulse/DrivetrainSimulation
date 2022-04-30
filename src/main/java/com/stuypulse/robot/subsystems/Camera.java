package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.DelayFilter;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.filters.IFilter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * A simulated camera.
 */
public class Camera extends SubsystemBase {
    
    private final Drivetrain drivetrain;
    private final Vector2D hub;

    private final IFilter angleMeasurement;
    private final IFilter distanceMeasurement;

    public Camera(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.hub = Field.HUB;

        this.angleMeasurement = new DelayFilter(Settings.Camera.DELAY);
        this.distanceMeasurement = new DelayFilter(Settings.Camera.DELAY);

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

        return ang.sub(hub.sub(pos).getAngle());
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
            return Angle.kZero;
        }

        return Angle.fromDegrees(angleMeasurement.get(getAngleToHub().toDegrees()));
    }

    public double getDistance() {
        if (!hasTarget()) {
            Settings.reportWarning("Unable to find Limelight target [getDistance()]");
            return 0;
        }

        return distanceMeasurement.get(getRawDistance());
    }

    /*
    public Angle getYAngle() {
        if (!hasTarget()) {
            Settings.reportWarning("Unable to find Limelight target [getYAngle()]");
            return Angle.kZero;
        }

        technically possible to work "backwards" using the distance and hub height 
        to find the angle to the horizontal and then subtract the horizontal angle of 
        the limelight 

        atan2(hub height, distance) - limelight_pitch
    }
    */

}
