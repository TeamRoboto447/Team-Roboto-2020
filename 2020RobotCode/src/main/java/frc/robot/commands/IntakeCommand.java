/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.controlmaps.OperatorMap;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class IntakeCommand extends CommandBase {
  IndexerSubsystem indexerSubsystem;
  TurretSubsystem turretSubsystem;
  /**
   * Creates a new IntakeCommand.
   */
  public IntakeCommand(IndexerSubsystem iSubsystem, TurretSubsystem tSubsystem) {
    this.indexerSubsystem = iSubsystem;
    this.turretSubsystem = tSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.operator.getRawButton(OperatorMap.X)) {
      runIntake();
    } else if(RobotContainer.operator.getRawButton(OperatorMap.RB)) {
      reverseIntake();
    } else {
      stopIntake();
    }

    if(RobotContainer.operator.getRawButton(OperatorMap.LT) && this.turretSubsystem.shooterAtSpeed()) {
      runIndexer();
    } else if(RobotContainer.operator.getRawButton(OperatorMap.LB)) {
      reverseIndexer();
    } else {
      stopIndexer();
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
    return false;
  }

  double systemSpeed = 0.5;

  private void runIntake() {
    double speed = -this.systemSpeed/2;
    this.indexerSubsystem.intakeRaw(speed);
  }

  private void reverseIntake() {
    double speed = this.systemSpeed/2;
    this.indexerSubsystem.intakeRaw(speed);
  }

  private void stopIntake() {
    this.indexerSubsystem.intakeRaw(0);
  }

  private void stopIndexer() {
    this.indexerSubsystem.indexerRaw(0);
  }

  private void reverseIndexer() {
    this.indexerSubsystem.indexerRaw(this.systemSpeed);
  }

  private void runIndexer() {
    this.indexerSubsystem.indexerRaw(-this.systemSpeed);
  }
}
