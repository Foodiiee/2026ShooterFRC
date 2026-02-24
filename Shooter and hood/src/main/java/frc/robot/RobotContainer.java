// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.HoodCommand;
import frc.robot.commands.IndexAndSpindexCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexAndSpindexSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class RobotContainer {
  public static final CommandXboxController driveController = new CommandXboxController(0);
  public static final CommandXboxController operatorController = new CommandXboxController(1);
  public ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public HoodSubsystem hoodSubsystem = new HoodSubsystem();
  public IndexAndSpindexSubsystem InSSubsystem = new IndexAndSpindexSubsystem();
  public static Trigger hasTarget;
  public DigitalInput hoodSwitch;

  public RobotContainer() {
    hoodSwitch = new DigitalInput(Constants.tempHoodSwitch);
    SmartDashboard.putNumber("flywheelSpeed", 0);
    SmartDashboard.putNumber("hood target position", 0);

    SmartDashboard.putNumber("kV", 0.1);
    SmartDashboard.putNumber("kP", 0.4);
    SmartDashboard.putNumber("kI", 0.0);
    SmartDashboard.putNumber("kD", 0.0);

    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    // CHANGE THIS THING TO ACTUAL HAS TARGET ONCE ERIC CODE IS DONE AND MERGED
    hasTarget = new Trigger(hoodSwitch::get);
    configureBindings();
  }

  private void configureBindings() {
    // Spin flywheel
    operatorController.a().onTrue(new ShooterCommand(shooterSubsystem, 20));

    // Manual hood override
    operatorController.povUp().onTrue(new HoodCommand(hoodSubsystem, true, 0.1));
    operatorController.povUp().onFalse(new HoodCommand(hoodSubsystem, true, 0));
    operatorController.povDown().onTrue(new HoodCommand(hoodSubsystem, true, -0.1));
    operatorController.povDown().onFalse(new HoodCommand(hoodSubsystem, true, 0));

    // Auto hood movement
    hasTarget.onFalse(new HoodCommand(hoodSubsystem, false, 0));

    // Regular Shooting
    driveController.x().onTrue(new IndexAndSpindexCommand(InSSubsystem, shooterSubsystem, false));

    // Force Shoot
    operatorController.b().onTrue(new IndexAndSpindexCommand(InSSubsystem, shooterSubsystem, true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
