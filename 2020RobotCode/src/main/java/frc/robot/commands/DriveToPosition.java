/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.subsystems.RobotDriveSubsystem;
import frc.robot.utils.MovingAverage;
import frc.robot.utils.PID;
import frc.robot.utils.ff.ConstantFF;

public class DriveToPosition extends CommandBase {

  private final MovingAverage averagePosition;

  private final PID drivePID;
  private final RobotDriveSubsystem driveSubsystem;
  private final double targetPosition;

  public DriveToPosition(RobotDriveSubsystem dSubsystem, double targetPosition) {
    this.driveSubsystem = dSubsystem;
    this.targetPosition = targetPosition;
    addRequirements(dSubsystem);

    this.drivePID = new PID(this.targetPosition, Constants.drivekP, Constants.drivekI, Constants.drivekD,
        new ConstantFF(0));
    this.averagePosition = new MovingAverage(50);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.driveSubsystem.setMotorIdleMode(IdleMode.kBrake);
    this.driveSubsystem.resetEncoders();
    this.driveSubsystem.setCurrentGear("low");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double PV = this.driveSubsystem.getAverageEncoderDistance();
    PV = Utilities.driveshaftIntputToOutput(PV, this.driveSubsystem.getCurrentGear());
    PV = Utilities.meterToEncoder(PV);

    this.averagePosition.push(PV);

    double leftSpeed = this.drivePID.run(PV), rightSpeed = this.drivePID.run(PV);
    this.driveSubsystem.tankDrive(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.targetPosition - 0.5 < this.averagePosition.getAverage()
        && this.averagePosition.getAverage() < this.targetPosition + 0.5;
  }
}
