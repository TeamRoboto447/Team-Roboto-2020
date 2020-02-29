/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class IntakeBalls extends CommandBase {
  private int ballCount = 0;
  private boolean wasLookingAtBall = false;
  private final IndexerSubsystem indexerSubsystem;
  public IntakeBalls(IndexerSubsystem iSubsystem) {
    this.indexerSubsystem = iSubsystem;
    addRequirements(iSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.indexerSubsystem.intakeBall();
    
    if(!wasLookingAtBall && this.indexerSubsystem.ballAtIntake()) {
      this.ballCount++;
      this.wasLookingAtBall = true;
    } else if(wasLookingAtBall && !this.indexerSubsystem.ballAtIntake()) {
      this.wasLookingAtBall = false;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.indexerSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.indexerSubsystem.isFull() || this.ballCount >= 3;
  }
}