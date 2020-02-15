/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Utilities;
import frc.robot.controlmaps.OperatorMap;
import frc.robot.subsystems.RobotDriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {
  TurretSubsystem turretSubsystem;
  RobotDriveSubsystem driveSubsystem;

  public TurretCommand(TurretSubsystem turretSub, RobotDriveSubsystem driveSub) {
    this.turretSubsystem = turretSub;
    this.driveSubsystem = driveSub;

    addRequirements(this.turretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.operator.getRawButton(OperatorMap.A)) {
      target();
    } else if (RobotContainer.operator.getRawButton(OperatorMap.B)) {
      if (this.turretSubsystem.validTarget) {
        target();
      } else {
        turnManual();
      }
    } else {
      turretLock();
    }

    if (RobotContainer.operator.getRawButton(OperatorMap.X)) {
      double testSpeed = this.turretSubsystem.getSpeedFromDist();
      this.turretSubsystem.runShooterAtSpeed(testSpeed);
      if (this.turretSubsystem.shooterAtSpeed()) {
        this.turretSubsystem.feedShooter();
      } else {
        this.turretSubsystem.stopFeeder();
      }
    } else if (RobotContainer.operator.getRawButton(OperatorMap.Y)) {
      this.turretSubsystem.runShooterAtSpeed(this.turretSubsystem.getManualSpeed());
      if (this.turretSubsystem.shooterAtSpeed()) {
        this.turretSubsystem.feedShooter();
      } else {
        this.turretSubsystem.stopFeeder();
      }
    } else {
      this.turretSubsystem.runShooterRaw(0);
      this.turretSubsystem.stopFeeder();
    }

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

  private void turretLock() {
    // int lockPosition = Math.round(this.driveSubsystem.getAngle() -
    // this.turretSubsystem.lastTargetPos);
    int lockPosition = 0;
    this.turretSubsystem.turnToAngle(lockPosition);
  }

  private void turnManual() {
    double position = -Utilities.adjustForDeadzone(RobotContainer.operator.getRawAxis(OperatorMap.lJoyX), 0.025);
    double angle = 180 * position;
    this.turretSubsystem.turnToAngle(angle, 5);
  }

  private void target() {
    this.turretSubsystem.turnToTarget();
  }
}
