package com.stuypulse.robot.subsystems;


import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.math.Angle;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    
    private double motorSpeed;
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
        this.motorSpeed = 0.0;
        
        this.encoder = new Encoder(Ports.Turret.LEFT, Ports.Turret.RIGHT);

        encoder.setDistancePerPulse(Settings.Motion.Encoders.TURRET_DISTANCE_PER_PULSE);

        this.encoderSim = new EncoderSim(encoder);
        encoderDistance = 0;
        
        this.sim = new FlywheelSim(
                LinearSystemId.identifyVelocitySystem(
                    Settings.SysID.Turret.kV,
                    Settings.SysID.Turret.kA
                ),
                DCMotor.getNEO(1),
                Settings.Motion.Encoders.GearRatio.TURRET_GEARING
            );

        this.controller =
            new PIDController(Settings.PID.Turret.kP, Settings.PID.Turret.kI, Settings.PID.Turret.kD);

        this.drivetrain = drivetrain;
        this.turretSim = drivetrain.getField().getObject("turret");

        target = Field.HUB;

        aligning = false;
    }

    public void setAlign(boolean aligning) {
        this.aligning = aligning;
    }

    // Motor and Encoder Functions

    private void setMotorSpeed(double speed) {
        this.motorSpeed = speed;
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

    // Periodic Functions

    @Override
    public void simulationPeriodic() {
        sim.setInputVoltage(motorSpeed);

        sim.update(Settings.Motion.DT);

        double rate = sim.getAngularVelocityRPM();
        encoderSim.setRate(rate);
        encoderDistance += rate * Settings.Motion.DT;

        encoderSim.setDistance(encoderDistance);
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
        if (projectedMotorDist > Settings.Turret.MAX_TURN_ANGLE || projectedMotorDist < Settings.Turret.MIN_TURN_ANGLE) {
            // invert angle
            // ex. -135 -> 225, 10 -> -350
            error = Math.signum(-error) * (360 - Math.abs(error));
        }

        setMotorSpeed(controller.update(error));
        
        setPose(Angle.fromDegrees(encoder.getDistance()));
    }

}
