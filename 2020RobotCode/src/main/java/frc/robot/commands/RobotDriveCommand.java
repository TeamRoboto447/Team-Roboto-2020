/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotDriveSubsystem;
import frc.robot.controlmaps.OperaterMap;

public class RobotDriveCommand extends CommandBase {
  /**
   * Creates a new RobotDriveCommand.
   */
  private RobotDriveSubsystem driveSubsystem;
  public RobotDriveCommand(RobotDriveSubsystem subsystem) {
    this.driveSubsystem = subsystem;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.driveSubsystem.tankDrive(RobotContainer.driverLeft.getY(), RobotContainer.driverRight.getY());
    this.driveSubsystem.setInvertedDrive(RobotContainer.driverRight.getRawButton(1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
