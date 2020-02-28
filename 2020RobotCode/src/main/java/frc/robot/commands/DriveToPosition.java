/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Utilities;
import frc.robot.subsystems.RobotDriveSubsystem;
import frc.robot.utils.PID;
import frc.robot.utils.ff.ConstantFF;

public class DriveToPosition extends CommandBase {

  private final PID drivePID;
  private double kP, kI, kD = 0;

  private final RobotDriveSubsystem driveSubsystem;
  private final double targetPosition;

  public DriveToPosition(RobotDriveSubsystem dSubsystem, double targetPosition) {
    this.driveSubsystem = dSubsystem;
    this.targetPosition = targetPosition;
    addRequirements(dSubsystem);
    this.drivePID = new PID(this.targetPosition, this.kP, this.kI, this.kD, new ConstantFF(0));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.driveSubsystem.resetEncoders();
    this.driveSubsystem.setCurrentGear("low");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double PV = this.driveSubsystem.getAverageEncoderDistance();
    PV = Utilities.driveshaftIntputToOutput(PV, this.driveSubsystem.getCurrentGear());
    PV = Utilities.meterToEncoder(PV);
    double
      leftSpeed = this.drivePID.run(PV),
      rightSpeed = this.drivePID.run(PV);
    this.driveSubsystem.tankDrive(leftSpeed, rightSpeed);
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
