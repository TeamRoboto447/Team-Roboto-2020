/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.RobotDriveSubsystem;

public class RobotDriveCommand extends CommandBase {
  private final RobotDriveSubsystem driveSubsystem;
  NetworkTableInstance table;
  public RobotDriveCommand(RobotDriveSubsystem subsystem) {
    driveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    this.table = NetworkTableInstance.getDefault();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // boolean driveInverted = RobotContainer.driverLeft.getRawButton(1);

    double rightSpeed = RobotContainer.driverLeft.getY();
    double leftSpeed = RobotContainer.driverRight.getY();
    driveSubsystem.tankDrive(-leftSpeed, -rightSpeed, this.driveSubsystem.getDriveInverted());
    
    double testSpeed;
    if(RobotContainer.operator.getRawButton(2)) {
      testSpeed = -1;
    } else if(RobotContainer.operator.getRawButton(3)) {
      testSpeed = this.table.getTable("chameleon-vision").getEntry("shooterSpeed").getDouble(-0.90);
    } else if(RobotContainer.operator.getRawButton(1)) {
      testSpeed = 0.25;
    }else {
      testSpeed = 0;
      this.driveSubsystem.resetShooterIntegral();
    }
    driveSubsystem.testMotor(testSpeed);

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
