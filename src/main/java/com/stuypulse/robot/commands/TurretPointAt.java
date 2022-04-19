package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.constants.Ports.Drivetrain;
import com.stuypulse.robot.subsystems.Camera;
import com.stuypulse.robot.subsystems.Turret;
import com.stuypulse.stuylib.control.Controller;
import com.stuypulse.stuylib.control.PIDController;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TurretPointAt extends InstantCommand {

    private final Turret turret;
    private final Vector2D target;

    public TurretPointAt(Turret turret, Vector2D target) {
        this.turret = turret;
        this.target = target;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.pointAt(target);
    }
}