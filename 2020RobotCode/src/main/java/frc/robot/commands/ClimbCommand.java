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
import frc.robot.subsystems.ClimberSubsystem;

public class ClimbCommand extends CommandBase {

  private final ClimberSubsystem climberSubsystem;
  public ClimbCommand(ClimberSubsystem cSubsystem) {
    this.climberSubsystem = cSubsystem;
    addRequirements(this.climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.operator.getPOV() == 0) {
      this.climberSubsystem.climberUnlock();
      this.climberSubsystem.climb(1);
    } else if(RobotContainer.operator.getRawButton(OperatorMap.back)) {
      this.climberSubsystem.climberUnlock();
      this.climberSubsystem.climb(-1);
    } else {
      this.climberSubsystem.climberLock();
      this.climberSubsystem.stop();
    }

    if(RobotContainer.operator.getPOV() == 180) {
      if(-RobotContainer.operator.getRawAxis(OperatorMap.rJoyY) > 0.5) {
        this.climberSubsystem.liftClimber();
      } else if(-RobotContainer.operator.getRawAxis(OperatorMap.rJoyY) < -0.5) {
        this.climberSubsystem.lowerClimber();
      }
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
}
