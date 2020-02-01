/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.*;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TurretCommand extends CommandBase {
  /**
   * Creates a new TurretCommand.
   */
  NetworkTableInstance table;
  NetworkTableEntry distanceEntry, shooterSpeed;
  TurretSubsystem turretSubsystem;
  public TurretCommand(TurretSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    turretSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.table = NetworkTableInstance.getDefault();
    this.distanceEntry = table.getTable("PID").getEntry("Distance");
    this.shooterSpeed = this.table.getTable("chameleon-vision").getEntry("shooterSpeed");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double testSpeed;
    //TODO adjust button maping
    if(RobotContainer.operator.getRawButton(2)) {
      testSpeed = 1;
    } else if(RobotContainer.operator.getRawButton(3)) {
      testSpeed = this.shooterSpeed.getDouble(0.90);
    } else if(RobotContainer.operator.getRawButton(1)) {
      testSpeed = -0.25;
    } else if(RobotContainer.operator.getRawButton(4)) {
      testSpeed = Constants.speedkM * this.distanceEntry.getDouble(0) + Constants.speedkB;
      this.shooterSpeed.setDouble(testSpeed);
    } else {
      testSpeed = 0;
      this.turretSubsystem.resetShooterIntegral();
    }

    this.turretSubsystem.runShooterAtSpeed(testSpeed);
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
