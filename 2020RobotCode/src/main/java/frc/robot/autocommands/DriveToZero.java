/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autocommands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.subsystems.RobotDriveSubsystem;
import frc.robot.utils.MovingAverage;
import frc.robot.utils.PID;
import frc.robot.utils.ff.ConstantFF;

public class DriveToZero extends CommandBase {

  private final MovingAverage averagePosition;

  private final PID drivePID, steerPID;
  private final RobotDriveSubsystem driveSubsystem;
  private final double minSpeed, maxSpeed;

  private boolean done = false;

  public DriveToZero(RobotDriveSubsystem dSubsystem, double minSpeed, double maxSpeed) {
    this.driveSubsystem = dSubsystem;
    this.minSpeed = minSpeed;
    this.maxSpeed = maxSpeed;
    addRequirements(dSubsystem);

<<<<<<< HEAD:2020RobotCode/src/main/java/frc/robot/commands/DriveToPosition.java
    this.drivePID = new PID(this.targetPosition, Constants.drivekP, Constants.drivekI, Constants.drivekD,
        new ConstantFF(0.25));
    this.averagePosition = new MovingAverage(50);
=======
    this.drivePID = new PID.PIDBuilder(0, Constants.drivekP, Constants.drivekI, Constants.drivekD)
        .FF(new ConstantFF(0.2)).Name("drive").build();
    this.steerPID = new PID.PIDBuilder(0, Constants.steerkP, Constants.steerkI, Constants.steerkD).FF(new ConstantFF(0))
        .Name("steer").build();
    this.averagePosition = new MovingAverage(5);
>>>>>>> e95d18fc533059d20f33108d343be5872c0d6767:2020RobotCode/src/main/java/frc/robot/autocommands/DriveToZero.java
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.driveSubsystem.setMotorIdleMode(IdleMode.kCoast);
    this.driveSubsystem.setCurrentGear("low");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double PV = this.driveSubsystem.getAverageEncoderDistance();
    PV = Utilities.driveshaftInputToOutput(PV, this.driveSubsystem.getCurrentGear());
    PV = Utilities.meterToEncoder(PV);

    this.averagePosition.push(PV);

    double speed = this.drivePID.run(PV);
    double steer = this.steerPID.run(this.driveSubsystem.getHeading());

    if (speed > this.maxSpeed) {
      speed = this.maxSpeed;
    } else if(speed < this.minSpeed) {
      speed = this.minSpeed;
    }

    if (!done) {
      this.driveSubsystem.arcadeDrive(speed, steer);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.done = true;
    this.driveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return 0.5 < this.averagePosition.getAverage()
        && this.averagePosition.getAverage() < 0.5;
  }
}
