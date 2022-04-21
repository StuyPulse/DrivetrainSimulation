package com.stuypulse.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
    
    private CANSparkMax motor;
    private Encoder encoder;
    private EncoderSim encoderSim;
    private FlywheelSim sim;

    private Controller controller;

    private Drivetrain drivetrain;
    private FieldObject2d turretSim;

    private Vector2D target;

    private double encoderDistance;

    public Turret(Drivetrain drivetrain) {
        this.motor = new CANSparkMax(Ports.Turret.MOTOR, MotorType.kBrushless);
        
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
    public void simulationPeriodic() {
        sim.setInputVoltage(motor.get());

        sim.update(Settings.Motion.DT);

        double rate = sim.getAngularVelocityRPM();
        encoderSim.setRate(rate);
        encoderDistance += rate * Settings.Motion.DT;

        encoderSim.setDistance(encoderDistance);
    }

    @Override
    public void periodic() {
        double error = getTargetAngle().toDegrees() - turretSim.getPose().getRotation().getDegrees();
    
        if (error < -180) {
            error += 360;
        } else if (error > 180) {
            error -= 360;
        }

        motor.set(controller.update(error));

        setPose(Angle.fromDegrees(encoder.getDistance()));
    }

}
