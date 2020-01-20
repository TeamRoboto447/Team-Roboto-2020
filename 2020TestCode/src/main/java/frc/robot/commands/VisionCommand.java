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

import frc.robot.utils.PID;
import frc.robot.utils.logging;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Utilities;
import frc.robot.subsystems.RobotDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends CommandBase {
  RobotDriveSubsystem driveSubsystem;
  VisionSubsystem visionSubsystem;
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
  double P, I, D, FF;
  //PID Variables
  PID turnPID, turnToBallPID;

  double ballX;
  
  boolean driveTargetCalculated = false;

  /**
   * Creates a new VisionCommand.
   */
  public VisionCommand(final RobotDriveSubsystem subsystem, final VisionSubsystem vSubsystem) {
    this.driveSubsystem = subsystem;
    this.visionSubsystem = vSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveSubsystem);
    addRequirements(this.visionSubsystem);

    this.table = NetworkTableInstance.getDefault();
    this.camInfo = this.table.getTable("chameleon-vision").getSubTable("Shooter Targeting");
    turnPID = new PID(0.0, 0.0207, 0.0414, 0.002588, 0.0, 0.0);
    //turnToBallPID = new PID(0.0, 0.003, 0.00433884297, 0.00051857142, 0.0, 0.0);
    turnToBallPID = new PID(0.0, 0.003, 0.0005, 0.0, 0.0, 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    getValues();
    updateTurningPIDValues();
    logging.info("\nTarget Valid: " + this.validTarget + "\nYaw: " + this.yaw + "\nPitch: " + this.pitch
        + "\nCamera Latency: " + this.latency, "shooterVision");
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

    this.visionSubsystem.setPixyLamp(false);
    if(RobotContainer.operator.getRawButton(1)) {
      turnToBall();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    this.turnPID.resetIntegral();
    this.visionSubsystem.reset();
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

    this.distance = (4.237885 * this.poseX) - 3.997617;

    this.PIDInfo = this.table.getTable("chameleon-vision").getSubTable("PID");
    this.PIDInfo.getEntry("Distance").setValue(this.distance);
    this.P = this.PIDInfo.getEntry("kP").getDouble(0.01986);
    this.I = this.PIDInfo.getEntry("kI").getDouble(0.070042);
    this.D = this.PIDInfo.getEntry("kD").getDouble(0.001408);
    this.FF = this.PIDInfo.getEntry("kFF").getDouble(0);

    this.ballX = this.table.getTable("PixyVision").getEntry("ballX").getDouble(-1000);
  }

  private void turnToTarget() {
    if (this.validTarget) {
      final double speed = turnPID.run(this.yaw, Robot.iterTime);
      logging.debug("Aiming PID Output Value: " + speed, "shooterVision");
      this.driveSubsystem.tankDrive(speed, -speed, false);
    } else {
      this.driveSubsystem.turnToZeroVeryInnacurate();
    }
  }

  private void driveToTarget() {
    if (this.validTarget) {
      logging.debug("Drive target calculated: "+this.driveTargetCalculated, "shooterVision");
      final double leftEncodePos = this.driveSubsystem.leftTalon.getSelectedSensorPosition();
      final double rightEncodePos = this.driveSubsystem.rightTalon.getSelectedSensorPosition();
      if(this.driveTargetCalculated) {

        this.driveSubsystem.driveToEncode(this.driveTargetLeft, this.driveTargetRight);
        logging.debug("Driving to distance", "shooterVision");
        logging.debug("Target encoder positions:\n    Left: "+this.driveTargetLeft+
          "\n    Right: "+this.driveTargetRight+
          "\nCurrent encoder positions:\n    Left: "+leftEncodePos+
          "\n    Right: "+rightEncodePos, "shooterVision");

      } else {
        this.driveSubsystem.leftTalon.setSelectedSensorPosition(0);
        this.driveSubsystem.rightTalon.setSelectedSensorPosition(0);
        double encodeFromTarget = Utilities.encoderToInch((this.distance - this.driveTargetInch)*12);
        logging.debug("Difference between target and distance: " + (this.distance - this.driveTargetInch), "shooterVision");
        this.driveTargetLeft = encodeFromTarget;
        this.driveTargetRight = encodeFromTarget;

        this.driveTargetCalculated = true;
      }
    }
  }

  public void updateTurningPIDValues() {
    logging.debug("Aiming PID Values: kP: "+ this.P +" kI: "+ this.I +"kD:"+this.D, "aimingPID");
    this.turnPID.updateP(this.P);
    this.turnPID.updateI(this.I);
    this.turnPID.updateD(this.D);
    this.turnPID.updateFF(this.FF, 0.0);
  }

  public void turnToBall() {
    //this.visionSubsystem.setPixyLamp(true);
    if(this.ballX != -1000) {
      if(this.ballX < -25 || this.ballX > 25) {
        double speed = this.turnToBallPID.run(this.ballX, Robot.iterTime);
        this.driveSubsystem.tankDrive(0.7+speed, 0.7-speed, false);
      } else {
        this.driveSubsystem.tankDrive(1, 1, false);
      }
    }
  }

  private void stop() {
    this.driveSubsystem.tankDrive(0, 0, false);
  }
}
