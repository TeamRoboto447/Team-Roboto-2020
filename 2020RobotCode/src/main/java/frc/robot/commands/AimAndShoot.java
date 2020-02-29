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
  private final Timer countDown, overallTimer;
  private final double maxRunTime;
  private double ballsShot = 0;
  private boolean wasLookingAtBall = false;

  public AimAndShoot(TurretSubsystem tSubsystem, IndexerSubsystem iSubsystem, double angle, double maxTime) {
    this.turretSubsystem = tSubsystem;
    this.indexerSubsystem = iSubsystem;
    this.startingAngle = angle;
    this.countDown = new Timer();
    this.countDown.stop();
    this.countDown.reset();
    this.overallTimer = new Timer();
    this.overallTimer.reset();
    this.overallTimer.start();

    this.maxRunTime = maxTime;

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
    this.turretSubsystem.enableTargetting(true);
    if (!this.turretSubsystem.validTarget) {
      this.turretSubsystem.turnToAngle(this.startingAngle, 0.5);
    } else {
      this.turretSubsystem.turnToTarget();
      if (this.turretSubsystem.onTarget()) {
        shoot();
      }
    }

    if (!this.wasLookingAtBall && this.indexerSubsystem.isFull()) {
      this.ballsShot++;
      this.wasLookingAtBall = true;
    } else if (this.wasLookingAtBall && !this.indexerSubsystem.isFull()) {
      this.wasLookingAtBall = false;
    }

    if(this.ballsShot >= 3) {
      this.countDown.reset();
      this.countDown.start();
    }
  }

  private void shoot() {
    double currentDistance = this.turretSubsystem.getDistance();
    double shooterSpeed = this.turretSubsystem.getSpeedFromDist(currentDistance);
    this.turretSubsystem.runShooterAtSpeed(shooterSpeed);
    if (this.turretSubsystem.shooterAtSpeed()) {
      this.turretSubsystem.feedShooter();
      this.indexerSubsystem.indexerRaw(0.75);
    } else {
      this.indexerSubsystem.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.enableTargetting(false);
    this.turretSubsystem.stop();
    this.indexerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.countDown.get() >= 0.5 || this.overallTimer.get() >= this.maxRunTime;
  }
}
