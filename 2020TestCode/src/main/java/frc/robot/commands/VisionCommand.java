/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.utils.PID;
import frc.robot.utils.logging;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Utilities;
import frc.robot.subsystems.RobotDriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.controlmaps.OperaterMap;

public class VisionCommand extends CommandBase {
  RobotDriveSubsystem driveSubsystem;
  VisionSubsystem visionSubsystem;
  TurretSubsystem turretSubsystem;

  double
    poseX,
    poseY,
    poseAngle,
    yaw,
    pitch,
    latency,
    distance;
  Boolean validTarget;
  NetworkTable camInfo, PIDInfo, PIDTuningInfo;
  NetworkTableInstance table;
  NetworkTableEntry validTargetEntry, yawEntry, pitchEntry, latencyEntry, targetPoseEntry, distanceEntry,
    PEntry, IEntry, DEntry, FFEntry, ballXEntry;

  //Vision Distance Values
  Double focal = 67.4799;
  Double thetaC = 0.585632;
  Double deltaHeight = 6.75;

  //PID Values
  //PID Tuning
  double target = 0;
  double driveTargetLeft, driveTargetRight;
  double driveTargetFeet = 11;
  double P, I, D, FF;
  //PID Variables
  PID turnPID, turnToBallPID;

  double ballX;
  
  boolean driveTargetCalculated = false;

  /**
   * Creates a new VisionCommand.
   */
  public VisionCommand(final RobotDriveSubsystem subsystem, final VisionSubsystem vSubsystem, final TurretSubsystem tSubsystem) {
    this.driveSubsystem = subsystem;
    this.visionSubsystem = vSubsystem;
    this.turretSubsystem = tSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveSubsystem);
    addRequirements(this.visionSubsystem);
    addRequirements(this.turretSubsystem);

    this.table = NetworkTableInstance.getDefault();
    this.camInfo = this.table.getTable("chameleon-vision").getSubTable("Shooter Targeting");
    turnPID = new PID(0.0, Constants.turnkP, Constants.turnkI, Constants.turnkD, Constants.turnkFFm, Constants.turnkFFb);
    turnToBallPID = new PID(0.0, Constants.turnToBallP, Constants.turnToBallI, Constants.turnToBallD, Constants.turnToBallFFm, Constants.turnToBallFFb);
    
    this.PIDInfo = this.table.getTable("PID");
    this.PIDTuningInfo = this.table.getTable("PIDTuning");
    this.validTargetEntry = this.camInfo.getEntry("isValid");
    this.yawEntry = this.camInfo.getEntry("targetYaw");
    this.pitchEntry = this.camInfo.getEntry("targetPitch");
    this.latencyEntry = this.camInfo.getEntry("latency");
    this.targetPoseEntry = this.camInfo.getEntry("targetPose");
    this.distanceEntry = this.PIDInfo.getEntry("Distance");
    this.PEntry = this.PIDTuningInfo.getEntry("turnkP");
    this.IEntry = this.PIDTuningInfo.getEntry("turnkI");
    this.DEntry = this.PIDTuningInfo.getEntry("turnkD");
    this.FFEntry = this.PIDTuningInfo.getEntry("turnkFF");
    this.ballXEntry = this.table.getTable("PixyVision").getEntry("ballX");
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
    if (RobotContainer.operator.getRawButton(OperaterMap.LT)) {
      this.driveTargetCalculated = false;
      turnToTarget();
    } else if(RobotContainer.operator.getRawButton(OperaterMap.RT)) {
      driveToTarget();
    } else {
      this.driveTargetCalculated = false;
      stop();
    }
    
    if(RobotContainer.operator.getRawButton(OperaterMap.RB)) {
      this.driveSubsystem.tankDrive(0.2, 0.2, false);
    }

    // this.visionSubsystem.setPixyLamp(false);
    if(RobotContainer.operator.getRawButton(OperaterMap.X)) {
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
    this.validTarget = this.validTargetEntry.getBoolean(false);
    this.yaw = this.yawEntry.getDouble(-1);
    this.pitch = this.pitchEntry.getDouble(-1);
    this.latency = this.latencyEntry.getDouble(-1);
    double defaultPose[] = new double[]{0, 0, 0};
    this.poseX = this.targetPoseEntry.getDoubleArray(defaultPose)[0];
    this.poseY = this.targetPoseEntry.getDoubleArray(defaultPose)[1];
    this.poseY = this.targetPoseEntry.getDoubleArray(defaultPose)[2];

    this.distance = this.distanceEntry.getDouble(-1);
    this.ballX = this.ballXEntry.getDouble(-1000);
  }

  private void turnToTarget() {
    if (this.validTarget) {
      double distanceToInner = this.getDistanceToInner(this.poseAngle, this.distance, Constants.distanceFromInnerToOuterPort);
      double adjustAngle = this.getAngleOffset(this.poseAngle, distanceToInner, Constants.distanceFromInnerToOuterPort);
      if (!Utilities.marginOfError(Constants.maxInnerPortAjustmentAngle, 0.0, adjustAngle)){
        adjustAngle = 0.0;
      }
      double targetPos = this.yaw + adjustAngle + Constants.turnTargetAjust; 
      double speed = turnPID.run(targetPos);
      logging.debug("Aiming PID Output Value: " + speed + "\nAiming PV: " + targetPos, "aimingPID");
      this.driveSubsystem.tankDrive(speed, -speed, false);
    } else {
      this.driveSubsystem.turnToZeroVeryInnacurate();
    }
  }
  private double getDistanceToInner( double angle, double distance, double targetDelta ){
    return Math.sqrt(Math.pow(targetDelta, 2) + Math.pow(distance,2) - targetDelta*distance*Math.cos(angle));
  }
  private double getAngleOffset( double angle, double distance, double targetDelta){
    return Math.asin(targetDelta / distance * Math.sin(angle));
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
        double encodeFromTarget = Utilities.feetToEncoder(this.distance - this.driveTargetFeet);
        logging.debug("Difference between target and distance: " + (this.distance - this.driveTargetFeet), "shooterVision");
        this.driveTargetLeft = encodeFromTarget;
        this.driveTargetRight = encodeFromTarget;

        this.driveTargetCalculated = true;
      }
    }
  }

  public void updateTurningPIDValues() {
    this.P = this.PEntry.getDouble(Constants.turnkP);
    this.I = this.IEntry.getDouble(Constants.turnkI);
    this.D = this.DEntry.getDouble(Constants.turnkD);
    this.FF = this.FFEntry.getDouble(Constants.turnkFFm);

    logging.debug("Aiming PID Values: kP: "+ this.P +" kI: "+ this.I + " kD: "+this.D, "aimingPID");
    this.turnPID.updateP(this.P);
    this.turnPID.updateI(this.I);
    this.turnPID.updateD(this.D);
    this.turnPID.updateFF(this.FF, 0.0);
  }

  public void turnToBall() {
    if(this.ballX != -1000) {
      if(! Utilities.marginOfError(Constants.turnToBallThreshold, 0, this.ballX)) { //this.ballX < -25 || this.ballX > 25 //! Utilities.marginOfError(Constants.turnToBallThreshold, 0, this.ballX)
        double speed = this.turnToBallPID.run(this.ballX);
        logging.debug(speed+"", "turntoball");
        this.driveSubsystem.tankDrive(Constants.turnToBallBaseSpeed + speed, Constants.turnToBallBaseSpeed - speed, true);
      } else {
        this.driveSubsystem.tankDrive(1, 1, true);
      }
    }
  }

  private void stop() {
    this.driveSubsystem.tankDrive(0, 0, false);
  }
}
