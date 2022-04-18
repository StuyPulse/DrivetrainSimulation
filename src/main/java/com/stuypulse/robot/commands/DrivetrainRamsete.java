package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class DrivetrainRamsete extends RamseteCommand {
    public DrivetrainRamsete(Drivetrain drivetrain, Trajectory traj) {
        super(
            traj,
            drivetrain::getPose,
            new RamseteController(),
            new DifferentialDriveKinematics(Settings.Motion.TRACK_WIDTH),
            drivetrain::tankDriveKalman,
            drivetrain
        );
    }
}
