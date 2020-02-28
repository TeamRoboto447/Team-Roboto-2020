/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AimAndShoot extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final double startingAngle;
  private final Timer countdownToEnd;

  public AimAndShoot(TurretSubsystem tSubsystem, IndexerSubsystem iSubsystem, double angle) {
    this.turretSubsystem = tSubsystem;
    this.indexerSubsystem = iSubsystem;
    this.startingAngle = angle;

    addRequirements(tSubsystem);
    addRequirements(iSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!this.turretSubsystem.validTarget) {
      this.turretSubsystem.turnToAngle(this.startingAngle, 0.5);
    } else {
      this.turretSubsystem.turnToTarget();
      if (this.turretSubsystem.onTarget()) {
        shoot();
      }
    }
  }

  private void shoot() {
    double currentDistance = this.turretSubsystem.getDistance();
    double shooterSpeed = this.turretSubsystem.getSpeedFromDist(currentDistance);
    this.turretSubsystem.runShooterAtSpeed(shooterSpeed);
    if (this.turretSubsystem.shooterAtSpeed()) {
      this.turretSubsystem.feedShooter();
      this.indexerSubsystem.indexerRaw(0.75);
    }
  }

  private boolean ballsLeft() {
    return this.indexerSubsystem.isFull() || this.indexerSubsystem.ballAtIntake()
        || this.indexerSubsystem.ballAtPosOne();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return countdownToEnd();
  }

  private boolean countdownToEnd() {
    if(!ballsLeft()) {
      this.countdownToEnd.reset();
      this.countdownToEnd.start();
    } else {
      this.countdownToEnd.stop();
      this.countdownToEnd.reset();
    }

    if(this.countdownToEnd.get() == 1) {
      return true;
    }

    return false;
  }
}
