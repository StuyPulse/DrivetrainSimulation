package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.DrivetrainRamsete;
import com.stuypulse.robot.util.TrajectoryLoader;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Balls extends SequentialCommandGroup {

    private final String balls = "Balls/output/Balls.wpilib.json";

    public Balls(RobotContainer robot) {
        addCommands(
            new DrivetrainRamsete(robot.drivetrain, balls)
        );

    }
    
}
