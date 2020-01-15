/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Utilities;
import frc.robot.subsystems.RobotDriveSubsystem;

public class VisionCommand extends CommandBase {
  RobotDriveSubsystem driveSubsystem;
  Double
    poseX,
    poseY,
    yaw,
    pitch,
    latency,
    distance;
  Boolean validTarget;
  NetworkTable camInfo, PIDInfo;
  NetworkTableInstance table;

  //Vision Distance Values
  Double focal = 67.4799;
  Double thetaC = 0.585632;
  Double deltaHeight = 6.75;

  //PID Values
  //PID Tuning
  double target = 0;
  double driveTargetLeft, driveTargetRight;
  double driveTargetInch = 11;
  double P, I, D, driveP, driveI, driveD;
  double iterTime = 0.01;
  //PID Variables
  double integral, previousError, driveIntegral, previousDriveError = 0;

  boolean driveTargetCalculated = false;
  /**
   * Creates a new VisionCommand.
   */
  public VisionCommand(final RobotDriveSubsystem subsystem) {
    driveSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    this.table = NetworkTableInstance.getDefault();
    this.camInfo = this.table.getTable("chameleon-vision").getSubTable("Shooter Targeting");
    this.PIDInfo = this.table.getTable("chameleon-vision").getSubTable("PID");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getValues();
    Utilities.logging("\nTarget Valid: " + this.validTarget + "\nYaw: " + this.yaw + "\nPitch: " + this.pitch
        + "\nCamera Latency: " + this.latency, "INFO");
    if (RobotContainer.operator.getRawButton(7)) {
      this.driveTargetCalculated = false;
      turnToTarget();
    } else if(RobotContainer.operator.getRawButton(8)) {
      driveToTarget();
    } else {
      this.driveTargetCalculated = false;
      stop();
    }

    if(RobotContainer.operator.getRawButton(6)) {
      this.driveSubsystem.tankDrive(0.2, 0.2, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private void getValues() {
    this.validTarget = this.camInfo.getEntry("isValid").getBoolean(false);
    this.yaw = this.camInfo.getEntry("targetYaw").getDouble(-1);
    this.pitch = this.camInfo.getEntry("targetPitch").getDouble(-1);
    this.latency = this.camInfo.getEntry("latency").getDouble(-1);

    this.poseX = this.camInfo.getEntry("targetPose").getDoubleArray(new double[]{0, 0})[0];
    this.poseY = this.camInfo.getEntry("targetPose").getDoubleArray(new double[]{0, 0})[1];

    this.P = this.PIDInfo.getEntry("P").getDouble(0.0207);
    this.I = this.PIDInfo.getEntry("I").getDouble(0.0414);
    this.D = this.PIDInfo.getEntry("D").getDouble(0.002588);

    this.driveP = this.PIDInfo.getEntry("driveP").getDouble(0.03);
    this.driveI = this.PIDInfo.getEntry("driveI").getDouble(0);
    this.driveD = this.PIDInfo.getEntry("driveD").getDouble(0);
    
    /*
    double tangentR = this.yaw / this.focal;
    double thetaT = Math.atan(tangentR) + this.thetaC;
    double tangentT = Math.tan(thetaT);
    this.distance = this.deltaHeight / tangentT;
    */

    this.distance = (4.237885 * this.poseX) - 3.997617;
    this.PIDInfo.getEntry("Distance").setValue(this.distance);
  }

  private void turnToTarget() {
    if (this.validTarget) {
      final double speed = PID();
      Utilities.logging("Aiming PID Output Value: " + speed, "DEBUG");
      this.driveSubsystem.tankDrive(speed, -speed, false);
    } else {
      this.driveSubsystem.turnToZeroVeryInnacurate();
    }
  }

  private void driveToTarget() {
    if (this.validTarget) {
      Utilities.logging("Drive target calculated: "+this.driveTargetCalculated, "DEBUG");
      final double leftEncodePos = this.driveSubsystem.leftTalon.getSelectedSensorPosition();
      final double rightEncodePos = this.driveSubsystem.rightTalon.getSelectedSensorPosition();
      if(this.driveTargetCalculated) {

        this.driveSubsystem.driveToEncode(this.driveTargetLeft, this.driveTargetRight);
        Utilities.logging("Driving to distance", "INFO");
        Utilities.logging("Target encoder positions:\n    Left: "+this.driveTargetLeft+
          "\n    Right: "+this.driveTargetRight+
          "\nCurrent encoder positions:\n    Left: "+leftEncodePos+
          "\n    Right: "+rightEncodePos, "DEBUG");

      } else {
        this.driveSubsystem.leftTalon.setSelectedSensorPosition(0);
        this.driveSubsystem.rightTalon.setSelectedSensorPosition(0);
        double encodeFromTarget = Utilities.encoderToInch((this.distance - this.driveTargetInch)*12);
        Utilities.logging("Difference between target and distance: " + (this.distance - this.driveTargetInch), "DEBUG");
        this.driveTargetLeft = encodeFromTarget;
        this.driveTargetRight = encodeFromTarget;

        this.driveTargetCalculated = true;
      }
    }
  }

  public double PID() {
    final double error = this.target - this.yaw;
    this.integral += (error * this.iterTime);
    final double derivitive = (error - this.previousError) / this.iterTime;
    this.previousError = error;
    return this.P*error + this.I*this.integral + this.D*derivitive;
  }

  /*public double PIDDrive() {
    final double driveError = this.driveTarget - this.distance;
    this.driveIntegral += (driveError * this.iterTime);
    final double driveDerivitive = (driveError - this.previousDriveError) / this.iterTime;
    this.previousDriveError = driveError;
    return this.driveP*driveError + this.driveI*this.driveIntegral + this.driveD*driveDerivitive;
  }*/

  private void stop() {
    this.driveSubsystem.tankDrive(0, 0, false);
  }
}
