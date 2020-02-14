/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Utilities;
import frc.robot.subsystems.TestDriveSubsystem;

public class TestDriveCommand extends CommandBase {

  TestDriveSubsystem driveSubsystem;
  /**
   * Creates a new TestDriveCommand.
   */
  public TestDriveCommand(TestDriveSubsystem dSubsystem) {
    this.driveSubsystem = dSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.driveSubsystem.tankDriveVolts(-Utilities.adjustForDeadzone(RobotContainer.driverLeft.getY(), 0.05)*12, -Utilities.adjustForDeadzone(RobotContainer.driverRight.getY(), 0.05)*12);
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