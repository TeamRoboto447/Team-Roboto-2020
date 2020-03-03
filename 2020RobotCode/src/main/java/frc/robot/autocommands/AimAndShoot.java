/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class AimAndShoot extends CommandBase {
  private final TurretSubsystem turretSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final Timer countDown, overallTimer, distanceTimer;
  private final double maxRunTime;
  private final int ballsToShoot;
  private double scanAngle;
  private int ballsShot = 0;
  private boolean wasLookingAtBall = false;

  public AimAndShoot(TurretSubsystem tSubsystem, IndexerSubsystem iSubsystem, boolean scanRight, double maxTime,
      int ballsToShoot) {
    this.turretSubsystem = tSubsystem;
    this.indexerSubsystem = iSubsystem;

    this.distanceTimer = new Timer();
    this.distanceTimer.stop();
    this.distanceTimer.reset();

    this.countDown = new Timer();
    this.countDown.stop();
    this.countDown.reset();

    this.overallTimer = new Timer();
    this.overallTimer.reset();
    this.overallTimer.start();

    this.maxRunTime = maxTime;
    this.ballsToShoot = ballsToShoot;

    this.scanAngle = scanRight ? -45 : 45;

    addRequirements(tSubsystem);
    addRequirements(iSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  private boolean timerStarted = false;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.turretSubsystem.enableTargetting(true);
    if (!this.turretSubsystem.validTarget) {
      if (this.turretSubsystem.getTurretPos() < -40) {
        this.scanAngle = 45;
      } else if (this.turretSubsystem.getTurretPos() > 40) {
        this.scanAngle = -45;
      }

      this.turretSubsystem.turnToAngle(this.scanAngle, 0.5);
      spinUp();
    } else {
      this.turretSubsystem.turnToTarget();
      if (this.turretSubsystem.onTarget()) {
        if (!this.timerStarted) {
          this.distanceTimer.start();
          this.timerStarted = true;
        }
        if (this.distanceTimer.get() > 0.2) {
          this.turretSubsystem.lockDistance();
          shoot();
        }
      } else {
        this.turretSubsystem.unlockDistance();
        // spinUp();
      }
    }

    if (!this.wasLookingAtBall && this.indexerSubsystem.isFull()) {
      this.ballsShot++;
      this.wasLookingAtBall = true;
    } else if (this.wasLookingAtBall && !this.indexerSubsystem.isFull()) {
      this.wasLookingAtBall = false;
    }

    if (this.ballsShot >= this.ballsToShoot) {
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
      this.turretSubsystem.stopFeeder();
      this.indexerSubsystem.stop();
    }
  }

  private void spinUp() {
    this.turretSubsystem.runShooterAtSpeed(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.turretSubsystem.enableTargetting(false);
    this.turretSubsystem.enableTargetting(false);
    this.turretSubsystem.unlockDistance();
    this.turretSubsystem.stop();
    this.indexerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.countDown.get() >= 0.5 || this.overallTimer.get() >= this.maxRunTime;
  }
}
