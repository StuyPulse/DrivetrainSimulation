/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.DrivetrainReset;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

  private RobotContainer robot;
  private Command auto;

  /*************************/
  /*** ROBOT SCHEDULEING ***/
  /*************************/

  @Override
  public void robotInit() {
    robot = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /*********************/
  /*** DISABLED MODE ***/
  /*********************/

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /***********************/
  /*** AUTONOMOUS MODE ***/
  /***********************/  

  @Override
  public void autonomousInit() {
    auto = robot.getAutonomousCommand();

    if (auto != null) {
      auto.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  /*******************/
  /*** TELEOP MODE ***/
  /*******************/

  @Override
  public void teleopInit() {

    if (auto != null) {
      auto.cancel();
    }

    robot.turret.setAlign(true);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    robot.turret.setAlign(false);
  }

  /*****************/
  /*** TEST MODE ***/
  /*****************/

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

    new DrivetrainReset(robot.drivetrain).schedule(false);
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
