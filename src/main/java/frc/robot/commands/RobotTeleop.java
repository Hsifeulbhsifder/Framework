// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.subsystems.drive.Drive;

public class RobotTeleop extends CommandBase {

  private final Drive drive;

  /** Creates a new DriveTeleop. */
  public RobotTeleop() {
    // Use addRequirements() here to declare subsystem dependencies.
    drive = Drive.getInstance();

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setCurrentState(drive.DISABLED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (OI.DR.getLeftBumper())
      drive.setOpenLoopSpeedScaling(0.5);
    else
      drive.setOpenLoopSpeedScaling(1.0);

    if (OI.DR.getRightBumper())
      drive.setCurrentState(drive.TANK);
    else
      drive.setCurrentState(drive.ARCADE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setCurrentState(drive.DISABLED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
