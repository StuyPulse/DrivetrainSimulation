/************************ PROJECT DORCAS ************************/
/* Copyright (c) 2022 StuyPulse Robotics. All rights reserved.  */
/* This work is licensed under the terms of the MIT license.    */
/****************************************************************/

package com.stuypulse.robot;

import com.stuypulse.robot.commands.BadAlignment;
import com.stuypulse.robot.commands.DriveDistance;
import com.stuypulse.robot.commands.DrivetrainAlign;
import com.stuypulse.robot.commands.DrivetrainDrive;
import com.stuypulse.robot.commands.TurretPointAt;
import com.stuypulse.robot.commands.auton.Balls;
import com.stuypulse.robot.commands.auton.TomatoBalls;
import com.stuypulse.robot.constants.Field;
import com.stuypulse.robot.constants.Ports;
import com.stuypulse.robot.subsystems.Camera;
import com.stuypulse.robot.subsystems.Drivetrain;
import com.stuypulse.robot.subsystems.Turret;
import com.stuypulse.stuylib.input.Gamepad;
import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.stuypulse.stuylib.math.Vector2D;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  // Subsystem
  public final Drivetrain drivetrain = new Drivetrain();
  public final Turret turret = new Turret(drivetrain);


  public final Camera camera = new Camera(drivetrain);


  // Gamepads
  public final Gamepad driver = new AutoGamepad(Ports.Gamepad.DRIVER);
  public final Gamepad operator = new AutoGamepad(Ports.Gamepad.OPERATOR);

  // Autons
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();

  // Robot container

  public RobotContainer() {
    configureDefaultCommands();
    configureButtonBindings();
    configureAutons();
  }

  /****************/
  /*** DEFAULTS ***/
  /****************/

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new DrivetrainDrive(drivetrain, driver));
  }

  /***************/
  /*** BUTTONS ***/
  /***************/

  private void configureButtonBindings() {
    driver.getRightButton().whileHeld(new BadAlignment(drivetrain, camera));
    driver.getLeftButton().whileHeld(new DrivetrainAlign(drivetrain, camera));

    driver.getTopButton().whenPressed(new TurretPointAt(turret, Field.HUB));
    driver.getBottomButton().whenPressed(new TurretPointAt(turret, new Vector2D(0, 0)));
  }

  /**************/
  /*** AUTONS ***/
  /**************/

  public void configureAutons() {
    autonChooser.addOption("PID", new DriveDistance.PID(drivetrain, 10));
    autonChooser.addOption("BangBang", new DriveDistance.BangBang(drivetrain, 10));
    autonChooser.addOption("Balls", new Balls(this));
    autonChooser.setDefaultOption("TomatoBalls", new TomatoBalls(this));

    SmartDashboard.putData("Autonomous", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
