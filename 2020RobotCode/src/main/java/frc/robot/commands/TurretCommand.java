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
import frc.robot.controlmaps.OperatorMap;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCommand extends CommandBase {
  TurretSubsystem turretSubsystem;

  public TurretCommand(TurretSubsystem turretSub) {
    this.turretSubsystem = turretSub;

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

    if (RobotContainer.operator.getRawButton(OperatorMap.RT)) {
      this.turretSubsystem.enableShooterLogging(true);
      shootAtDistance();
      runFeeder();
    } else if (RobotContainer.operator.getRawButton(OperatorMap.Y)) {
      this.turretSubsystem.enableShooterLogging(true);
      shootAtSpeed();
      runFeeder();

    } else if(RobotContainer.operator.getRawButton(OperatorMap.start)) {
      this.turretSubsystem.enableShooterLogging(true);
      shootAtSpeed();
      this.turretSubsystem.feedShooter();

    } else {
      this.turretSubsystem.enableShooterLogging(false);
      this.turretSubsystem.runShooterRaw(0);
      this.turretSubsystem.stopFeeder();
    }

    if(RobotContainer.operator.getRawButton(OperatorMap.LT)) {
      lockDist();
    } else {
      unlockDist();
    }

  }

  private void lockDist() {
    this.turretSubsystem.lockDistance();
  }

  private void unlockDist() {
    this.turretSubsystem.unlockDistance();
  }

  private void shootAtDistance() {
    double dist = this.turretSubsystem.getDistance();
    double speed = this.turretSubsystem.getSpeedFromDist(dist);
    this.turretSubsystem.runShooterAtSpeed(speed);
  }

  private void shootAtSpeed() {
    double speed = this.turretSubsystem.getManualSpeed();
    this.turretSubsystem.runShooterAtSpeed(speed);
  }

  private void runFeeder() {
    if (this.turretSubsystem.shooterAtSpeed() && RobotContainer.operator.getRawButton(OperatorMap.LT)) {
      this.turretSubsystem.feedShooter();
    } else {
      this.turretSubsystem.stopFeeder();
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void turretLock() {
    this.turretSubsystem.enableTargetting(false);
    int lockPosition = 0;
    this.turretSubsystem.turnToAngle(lockPosition);
  }

  private void turnManual() {
    this.turretSubsystem.enableTargetting(true);
    double position = -Utilities.adjustForDeadzone(RobotContainer.operator.getRawAxis(OperatorMap.lJoyX), 0.025);
    double angle = 180 * position;
    this.turretSubsystem.turnToAngle(angle, 0.5);
  }

  private void target() {
    this.turretSubsystem.enableTargetting(true);
    this.turretSubsystem.turnToTarget();
  }
}
