package com.stuypulse.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class NoMotor implements MotorController, Sendable {

    private double targetSpeed;

    
    public NoMotor() {
        targetSpeed = 0;
    }


    public double get() {
        return targetSpeed;
    }

    public void set(double speed) {
        targetSpeed = speed;
    }

    public void disable() {
        set(0);
    }

    public void stopMotor() {
        set(0);
    }


    public boolean getInverted() {
        return false;
    }

    public void setInverted(boolean inverted) {}


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("FakeMotor");
        builder.setSafeState(this::disable);
        builder.addDoubleProperty("setpoint", this::get, this::set);
    }

}
