package com.stuypulse.robot.commands;

import com.stuypulse.stuylib.control.BangBangController;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Drivetrain;

public class DriveDistance extends CommandBase {
    public static class PID extends DriveDistance {
        public PID(Drivetrain drivetrain, double distance) {
            super(drivetrain, distance, new PIDController(Settings.PID.kP, Settings.PID.kI, Settings.PID.kD));
        }
    }

    public static class BangBang extends DriveDistance {
        public BangBang(Drivetrain drivetrain, double distance) {
            super(drivetrain, distance, new BangBangController(Settings.BangBang.SPEED.get()));
        }
    }

    private final Drivetrain drivetrain;
    private final double distance;
    private final Controller controller;

    private double targetDistance;

    public DriveDistance(Drivetrain drivetrain, double distance, Controller controller) {
        this.drivetrain = drivetrain;
        this.distance = distance;
        this.controller = controller;
        
        targetDistance = 0.0;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        targetDistance = this.distance + drivetrain.getDistance();
    }
    
    @Override
    public void execute() {
        double error = targetDistance - drivetrain.getDistance();
        SmartDashboard.putNumber("Error", error);
        
        drivetrain.arcadeDrive(controller.update(error), 0.0);
    }

    @Override
    public boolean isFinished() {
        return controller.isDone(1e-2);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.arcadeDrive(0, 0);
    }
}