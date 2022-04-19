package com.stuypulse.robot.commands.auton;

import com.stuypulse.robot.RobotContainer;
import com.stuypulse.robot.commands.DrivetrainRamsete;
import com.stuypulse.robot.commands.DrivetrainReset;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TomatoBalls extends SequentialCommandGroup {

    private final String first_path = "TomatoBalls/output/Unnamed.wpilib.json";
    private final String second_path = "TomatoBalls/output/Unnamed_0.wpilib.json";
    private final String third_path = "TomatoBalls/output/Unnamed_1.wpilib.json";


    public TomatoBalls(RobotContainer robot) {
        addCommands(
            new DrivetrainRamsete( robot.drivetrain, first_path)
            .robotRelative(),
            new DrivetrainRamsete(robot.drivetrain, second_path)
            .fieldRelative(),
            new DrivetrainRamsete(robot.drivetrain, third_path)
            .fieldRelative()
        );
    }
    
}
