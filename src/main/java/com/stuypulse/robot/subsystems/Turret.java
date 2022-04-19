package com.stuypulse.robot.subsystems;

import com.stuypulse.robot.constants.Settings.PID.DriveDistance;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    
    private Mechanism2d turret;
    private MechanismRoot2d root;
    private Drivetrain drivetrain;


    public Turret (Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        Pose2d robotPos = drivetrain.getPose();

        this.turret = new Mechanism2d(3,3);
        this.root = turret.getRoot("turret", robotPos.getX() , robotPos.getY());

        // put on the SD
        SmartDashboard.putData("turret", turret);

    }

    @Override
    public void periodic() {
        root.setPosition(drivetrain.getPose().getX(), drivetrain.getPose().getY());
    }



}
