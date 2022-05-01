package com.stuypulse.robot.subsystems;


import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.NoMotor;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Turret extends SubsystemBase {

    private NoMotor motor;
    private Encoder encoder;
    private EncoderSim encoderSim;
    private FlywheelSim sim;

    private Controller controller;

    private Drivetrain drivetrain;
    private FieldObject2d turretSim;

    private Vector2D target;

    private double encoderDistance;

    private boolean aligning;

    public Turret(Drivetrain drivetrain) {
        this.motor = new NoMotor();
        
        this.encoder = new Encoder(Ports.Turret.LEFT, Ports.Turret.RIGHT);

        encoder.setDistancePerPulse(Settings.Turret.TURRET_DISTANCE_PER_PULSE);

        this.encoderSim = new EncoderSim(encoder);
        encoderDistance = 0;
        
        this.sim = Settings.Turret.System.getSystem();
        this.controller = Settings.Turret.Turning.getController();

        this.drivetrain = drivetrain;
        this.turretSim = drivetrain.getField().getObject("turret");

        target = Field.HUB;

        aligning = false;
    }

    public void setAlign(boolean aligning) {
        this.aligning = aligning;
    }

    // Angle and Position Functions

    public void pointAt(Vector2D target) {
        this.target = target;
    }

    private void setPose(Angle angle) {
        Pose2d robotPose = drivetrain.getPose();
        turretSim.setPose(new Pose2d(robotPose.getTranslation(), Rotation2d.fromDegrees(angle.toDegrees() + robotPose.getRotation().getDegrees())));
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

    private double getRawAngle() {
        return encoder.getDistance();
    }

    // Periodic Functions

    @Override
    public void simulationPeriodic() {
        sim.setInputVoltage(motor.get());

        sim.update(Settings.dT);

        double rate = sim.getAngularVelocityRPM();
        encoderSim.setRate(rate);
        encoderDistance += rate * Settings.dT;

        encoderSim.setDistance(encoderDistance);
    }

    private static double benvert(double angle) {
        return Math.signum(-angle) * (360 - Math.abs(angle));
    }
        
    @Override
    public void periodic() {
        if (!aligning) {
            return;
        }

        Angle toTarget = getTargetAngle();
        Angle turretFacing = Angle.fromDegrees(turretSim.getPose().getRotation().getDegrees());

        double error = toTarget.sub(turretFacing).toDegrees();

        double projectedMotorDist = error + encoder.getDistance();

        // if motor is going to overrun
        if (projectedMotorDist > Settings.Turret.Turning.MAX_ANGLE || 
            projectedMotorDist < Settings.Turret.Turning.MIN_ANGLE) {
            
            // invert angle
            // ex. -135 -> 225, 10 -> -350
            // projectedMotorDist = benvert(projectedMotorDist);
            // error = 0;
            projectedMotorDist = encoder.getDistance();
        }

        error = projectedMotorDist - encoder.getDistance();

        motor.set(controller.update(error));
        
        setPose(Angle.fromDegrees(encoder.getDistance()));
    }

}
