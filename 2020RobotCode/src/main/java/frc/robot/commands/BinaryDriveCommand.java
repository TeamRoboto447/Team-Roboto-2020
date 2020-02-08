/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.controlmaps.OperaterMap;
import frc.robot.subsystems.RobotDriveSubsystem;

public class BinaryDriveCommand extends CommandBase {

  private RobotDriveSubsystem driveSubsystem;

  /**
   * Creates a new BinaryDriveCommand.
   */
  public BinaryDriveCommand(RobotDriveSubsystem subsystem) {
    this.driveSubsystem = subsystem;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.operator.getRawButton(OperaterMap.X)){ 
      double distance = this.getNumber(1);
      this.driveSubsystem.driveToFeet(distance);
    } else if (RobotContainer.operator.getRawButton(OperaterMap.A)) {
      this.driveSubsystem.setInvertedDrive(false);
      double speed = this.getNumber(1.0 / 15);
      this.driveSubsystem.arcadeDrive(speed, RobotContainer.operator.getRawAxis(OperaterMap.lJoyX)/2);
    } else if (RobotContainer.operator.getRawButton(OperaterMap.B)) {
      this.driveSubsystem.setInvertedDrive(true);
      double speed = this.getNumber(1.0 / 15);
      this.driveSubsystem.arcadeDrive(speed, RobotContainer.operator.getRawAxis(OperaterMap.lJoyX)/2);
    } else{
      this.driveSubsystem.setInvertedDrive(RobotContainer.driverRight.getRawButton(1));
      this.driveSubsystem.tankDrive(RobotContainer.driverLeft.getY(), RobotContainer.driverRight.getY());
    }
  }
  private double getNumber(double base){
    double out=0.0;
    if (RobotContainer.operator.getRawButton(OperaterMap.RT)){
      out += Math.pow(2,0)*base;
    }
    if(RobotContainer.operator.getRawButton(OperaterMap.RB)){
      out += Math.pow(2,1)*base;
    }
    if(RobotContainer.operator.getRawButton(OperaterMap.LB)){
      out += Math.pow(2,2)*base;
    }
    if(RobotContainer.operator.getRawButton(OperaterMap.LT)){
      out += Math.pow(2,3)*base;
    }
    return out;
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
