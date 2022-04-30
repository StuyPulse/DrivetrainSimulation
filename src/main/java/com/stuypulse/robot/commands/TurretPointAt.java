package com.stuypulse.robot.commands;

import com.stuypulse.robot.subsystems.Turret;
import com.stuypulse.stuylib.math.Vector2D;
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