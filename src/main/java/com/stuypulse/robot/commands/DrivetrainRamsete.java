package com.stuypulse.robot.commands;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.Drivetrain;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class DrivetrainRamsete extends RamseteCommand {
    Drivetrain drivetrain;
    Trajectory traj;
    boolean resetPosition;

    public DrivetrainRamsete(Drivetrain drivetrain, Trajectory traj) {
        super(
            traj,
            drivetrain::getPose,
            new RamseteController(),
            new DifferentialDriveKinematics(Settings.Motion.TRACK_WIDTH),
            drivetrain::tankDriveKalman,
            drivetrain
        );
        this.drivetrain = drivetrain;
        this.traj = traj;
        this.resetPosition = true;
    }


    // [DEFAULT] Resets the drivetrain to the begining of the trajectory
    public DrivetrainRamsete robotRelative() {
        this.resetPosition = true;
        return this;
    }

    // Make the trajectory relative to the field
    public DrivetrainRamsete fieldRelative() {
        this.resetPosition = false;
        return this;
    }

    @Override
    public void initialize() {
        
        drivetrain.getField().getObject("traj").setTrajectory(traj);

        super.initialize();
        if (resetPosition) drivetrain.reset(traj.getInitialPose());
    }
}
