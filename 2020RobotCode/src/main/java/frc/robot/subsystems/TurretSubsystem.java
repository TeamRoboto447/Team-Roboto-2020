/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Logging;
import frc.robot.utils.PID;
import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.utils.ff.ConstantFF;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import java.lang.Math;

public class TurretSubsystem extends SubsystemBase {
  /**
   * Creates a new TurretSubsystem.
   */
  NetworkTableInstance table;
  NetworkTable PIDInfo, pidTuningPVs, camInfo;
  NetworkTableEntry
  // Declare Shooter PID tuning entries
  shootPEntry, shootIEntry, shootDEntry, shootFFmEntry, shootFFbEntry, bypassShooterPIDEntry,
      // Declare Turret PID tuning entries
      turretPEntry, turretIEntry, turretDEntry, turretFFmEntry, turretFFbEntry,
      // Declare information entries
      shooterSpeedEntry, shooterCurrSpeedEntry, turretEncoderEntry, turretLastTargetEntry, turretLastTargetOffsetEntry, realDistanceEntry,
      // Declare targetting entries
      validTargetEntry, pitchEntry, latencyEntry, targetPoseEntry, distanceEntry, yawEntry;

  PID shootingMotorPID, turretPositionPID;
  Boolean bypassShooterPID;
  Double shootP, shootI, shootD, shootFFm, shootFFb;
  Double turretP, turretI, turretD;

  double turretOffset;

  double poseX, poseY, poseAngle, yaw, pitch, latency, distance;
  public Boolean validTarget;

  CANSparkMax turretMotor, shootingMotorLeft, shootingMotorRight;
  CANEncoder shooterEncoder, turretEncoder;

  RobotDriveSubsystem driveSubsystem;

  public TurretSubsystem(RobotDriveSubsystem driveSub) {
    this.driveSubsystem = driveSub;
    setupNetworkTables();
    setupMotorsAndEncoders();
    setupPIDControllers();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getValues();
    // notConnectedYet updateShooterPIDValues();
    updateTurretPIDValues();
    updateNetworkTables();
  }

  // Directly related to shooter
  double prevSetpoint = 0;

  public void runShooterAtSpeed(double speed) {
    // Target velocity: 4851
    // Max velocity: 5700
    double targetVel = speed * 5415;
    this.shooterSpeedEntry.setDouble(targetVel);
    this.shootingMotorPID.updateSetpoint(targetVel);

    double delta = this.prevSetpoint - targetVel;
    if (delta > 100 || delta < -100) {
      this.shootingMotorPID.resetIntegral();
    }

    double workingSpeed = this.shootingMotorPID.run(this.shooterEncoder.getVelocity());
    if (workingSpeed < 0) {
      workingSpeed = 0;
    }

    if (speed <= 0) {
      workingSpeed = speed;
    }
    if (this.bypassShooterPID) {
      workingSpeed = speed; // Bypass PID if it's going nuts
    }

    shootingMotorLeft.set(workingSpeed);
    shootingMotorRight.set(-workingSpeed);

    
    Logging.debug("FF set: " + targetVel + ", FF out: " +
     this.shooterEncoder.getVelocity(), "FFTuning");
     
    Logging.debug("Input:" + workingSpeed, "shooterPID");
    Logging.debug("Shooter:" + this.shooterEncoder.getVelocity(), "shooterPID");
    Logging.debug("Target:" + targetVel, "shooterPID");
    
    this.prevSetpoint = targetVel;
  }

  public void resetShooterIntegral() {
    this.shootingMotorPID.resetIntegral();
  }

  public void updateShooterPIDValues() {
    this.shootP = this.shootPEntry.getDouble(Constants.shooterkP);
    this.shootI = this.shootIEntry.getDouble(Constants.shooterkI);
    this.shootD = this.shootDEntry.getDouble(Constants.shooterkD);
    this.shootFFm = this.shootFFmEntry.getDouble(Constants.shooterkFFb);
    this.shootFFb = this.shootFFbEntry.getDouble(Constants.shooterkFFm);
    this.bypassShooterPID = this.bypassShooterPIDEntry.getBoolean(Constants.bypassShooterPID);

    Logging.debug("Shooter PID Values: kP: " + this.shootP + " kI: " +
      this.shootI + " kD: " + this.shootD, "shootingPID");

    shootingMotorPID.updateP(this.shootP);
    shootingMotorPID.updateI(this.shootI);
    shootingMotorPID.updateD(this.shootD);
    //shootingMotorPID.updateFF(this.shootFFm, this.shootFFb);
  }

  public void setTurretTarget(double angle) {
    this.turretPositionPID.updateSetpoint(angle);
  }

  // Directly related to turret
  public void turnToAngle(double angle) {
    this.setTurretTarget(angle);
    double speed = this.turretPositionPID.run(this.getTurretPos());
    this.turretMotor.set(speed);

  }

  public void turnToAngle(double angle, double maxSpeed) {
    this.setTurretTarget(angle);
    double speed = this.turretPositionPID.run(this.getTurretPos());
    if (Math.abs(speed) > maxSpeed) {
      double sign = speed > 0 ? 1 : -1;
      speed = maxSpeed * sign;
    }
    this.turretMotor.set(speed);
  }

  public void turnRaw(double speed) {
    this.turretMotor.set(speed);
  }

  private double getDistanceToInner(double angle, double distance, double targetDelta) {
    return Math.sqrt(Math.pow(targetDelta, 2) + Math.pow(distance, 2) - targetDelta * distance * Math.cos(angle));
  }

  private double getAngleOffset(double angle, double distance, double targetDelta) {
    return Math.asin(targetDelta / distance * Math.sin(angle));
  }

  boolean pastLimit = false;
  public double lastTargetPos = 0;
  public void turnToTarget() {
    setTurretTarget(0);
    Logging.debug("Turret limit: "+Constants.turretSpinLimit+"\nTurret Position: "+this.getTurretPos()+"\nTurret Past Limit: "+this.pastLimit+"\nValid Target: "+this.validTarget, "turretLimits");
    if (!this.pastLimit && this.validTarget) {
      this.pastLimit = Math.abs(this.getTurretPos()) > Constants.turretSpinLimit;
      double distanceToInner = this.getDistanceToInner(this.poseAngle, this.distance,
          Constants.distanceFromInnerToOuterPort);
      double adjustAngle = this.getAngleOffset(this.poseAngle, distanceToInner, Constants.distanceFromInnerToOuterPort);
      if (!Utilities.marginOfError(Constants.maxInnerPortAjustmentAngle, 0.0, adjustAngle)) {
        adjustAngle = 0.0;
      }
      double processingVar = this.yaw;
      double speed = this.turretPositionPID.run(processingVar);

      Logging.debug("Aiming PID Output Value: " + speed + "\nAiming PV: " +
        processingVar, "aimingPID");

      this.turnRaw(speed);
      this.lastTargetPos = this.getTurretPos();
      this.turretOffset = ((this.driveSubsystem.getAngle()) - this.lastTargetPos) % 360;
      this.lastTargetPos += (turretOffset);
      this.lastTargetPos = this.clamp(this.lastTargetPos);
    } else if (!this.pastLimit) {/*
      this.pastLimit = Math.abs(this.getTurretPos()) > Constants.turretSpinLimit;
      if(this.startingTurretPos == -9000) {
        this.startingTurretPos = this.clamp(this.lastTargetPos);
      }
      double turretPos = this.clamp(this.getTurretPos());
      //double stabilizedPos = ((this.clamp(this.driveSubsystem.getAngle()) - (this.lastTargetPos - this.turretOffset))+180)%360;
      double subtractedVal = this.lastTargetPos < 0 ? (this.lastTargetPos - this.turretOffset) : (this.lastTargetPos + this.turretOffset);
      System.out.println(subtractedVal);
      double stabilizedPos = this.clamp(this.driveSubsystem.getAngle() - (180+subtractedVal));
      //System.out.println("Before: "+stabilizedPos);
      if(stabilizedPos > Constants.turretSpinLimit) {
        stabilizedPos -= 360;
      } else if(stabilizedPos < -Constants.turretSpinLimit) {
        stabilizedPos += 360;
      }
      //System.out.println("After: "+stabilizedPos+"\n");
      this.turnToAngle(stabilizedPos, 0.5);*/
      double targetPos = this.lastTargetPos;
      Logging.debug("Turret Position: "+this.getTurretPos()+"\nTarget Position: "+targetPos, "turret180");
      this.turnToAngle(this.lastTargetPos, 0.5);

    } else if (this.pastLimit) {
      this.pastLimit = Math.abs(this.getTurretPos()) > 45;
      double targetPos = this.lastTargetPos;
      Logging.debug("Turret Position: "+this.getTurretPos()+"\nTarget Position: "+targetPos, "turret180");
      this.turnToAngle(this.lastTargetPos, 0.75);
    }
    //System.out.println(this.pastLimit);

  }

  private double getTurretPos() {
    return this.turretEncoder.getPosition() * 3.6;
  }

  public void updateTurretPIDValues() {
    this.turretP = this.turretPEntry.getDouble(Constants.turretkP);
    this.turretI = this.turretIEntry.getDouble(Constants.turretkI);
    this.turretD = this.turretDEntry.getDouble(Constants.turretkD);

    this.turretPositionPID.updateP(this.turretP);
    this.turretPositionPID.updateI(this.turretI);
    this.turretPositionPID.updateD(this.turretD);

    Logging.debug("Turret PID Values: kP: " + this.turretPositionPID.getP() + " kI: " +
    this.turretPositionPID.getI() + " kD: " + this.turretPositionPID.getD(), "turretPID");
  }

  // Related to everything
  private void updateNetworkTables() {
    // notConnectedYet
    // this.shooterCurrSpeedEntry.setDouble(this.shooterEncoder.getVelocity());
    this.turretEncoderEntry.setDouble(this.getTurretPos()); // 360 degrees = 100
    this.distanceEntry.setDouble(this.distance);
  }

  private void setupMotorsAndEncoders() {
    // Set up Motors
    this.turretMotor = new CANSparkMax(Constants.turretSparkMax, MotorType.kBrushless);
    // notConnectedYet this.shootingMotorLeft = new
    // CANSparkMax(Constants.shooterSparkMaxLeft, MotorType.kBrushless);
    // notConnectedYet this.shootingMotorRight = new
    // CANSparkMax(Constants.shooterSparkMaxRight, MotorType.kBrushless);

    // Set up Encoders
    this.turretEncoder = this.turretMotor.getEncoder();
    // notConnectedYet this.shooterEncoder = this.shootingMotorLeft.getEncoder();\

    this.turretEncoder.setPosition(0);
    // notConnectedYet this.shooterEncoder.setPosition(0);
  }

  private void setupPIDControllers() {
    // Define PID controllers
    // notConnectedYet this.shootingMotorPID = new PID(0, // Default setpoint
    // notConnectedYet Constants.shooterkP,
    // notConnectedYet Constants.shooterkI,
    // notConnectedYet Constants.shooterkD,
    // notConnectedYet Constants.shooterkFFm,
    // notConnectedYet Constants.shooterkFFb,
    // notConnectedYet -Constants.shooterIZone, // Minimum integral
    // notConnectedYet Constants.shooterIZone); // Maximum integral
  
    this.turretPositionPID = new PID(0, // Default setpoint
        Constants.turretkP, Constants.turretkI, Constants.turretkD, new ConstantFF(0.0),
        -Constants.turretIZone, // Minimum integral
        Constants.turretIZone); // Maximum integral
}

  private void setupNetworkTables() {
    this.table = NetworkTableInstance.getDefault();
    this.PIDInfo = this.table.getTable("PID");
    this.pidTuningPVs = this.table.getTable("pidTuningPVs");
    this.camInfo = this.table.getTable("chameleon-vision").getSubTable("Shooter Targeting");

    // Define Monitor entries
    this.shooterSpeedEntry = this.PIDInfo.getEntry("shootTargetSpeed");
    this.shooterCurrSpeedEntry = this.pidTuningPVs.getEntry("Shooter Speed");
    this.turretEncoderEntry = this.pidTuningPVs.getEntry("Turret Position");

    // Define Shooter PID entries
    this.shootPEntry = this.PIDInfo.getEntry("shootkP");
    this.shootIEntry = this.PIDInfo.getEntry("shootkI");
    this.shootDEntry = this.PIDInfo.getEntry("shootkD");
    this.shootFFmEntry = this.PIDInfo.getEntry("shootkFFm");
    this.shootFFbEntry = this.PIDInfo.getEntry("shootkFFb");
    this.bypassShooterPIDEntry = this.PIDInfo.getEntry("bypassShooterPID");

    // Define turret PID entries
    this.turretPEntry = this.PIDInfo.getEntry("turretkP");
    this.turretIEntry = this.PIDInfo.getEntry("turretkI");
    this.turretDEntry = this.PIDInfo.getEntry("turretkD");

    // Define targetting entries
    this.validTargetEntry = this.camInfo.getEntry("isValid");
    this.yawEntry = this.camInfo.getEntry("targetYaw");
    this.pitchEntry = this.camInfo.getEntry("targetPitch");
    this.latencyEntry = this.camInfo.getEntry("latency");
    this.targetPoseEntry = this.camInfo.getEntry("targetPose");
    this.distanceEntry = this.pidTuningPVs.getEntry("Distance From Target");

    // Define other entries
    this.turretLastTargetEntry = this.pidTuningPVs.getEntry("Last Known Target Pos");
    this.turretLastTargetOffsetEntry = this.pidTuningPVs.getEntry("Offset From Last Known Target Pos");

    // Set default values
    this.shootPEntry.setDouble(Constants.shooterkP);
    this.shootIEntry.setDouble(Constants.shooterkI);
    this.shootDEntry.setDouble(Constants.shooterkD);
    this.shootFFmEntry.setDouble(Constants.shooterkFFm);
    this.shootFFbEntry.setDouble(Constants.shooterkFFb);
    this.bypassShooterPIDEntry.setBoolean(Constants.bypassShooterPID);
    this.shooterCurrSpeedEntry.setDouble(0);

    this.turretPEntry.setDouble(Constants.turretkP);
    this.turretIEntry.setDouble(Constants.turretkI);
    this.turretDEntry.setDouble(Constants.turretkD);
  }

  private void getValues() {
    this.validTarget = this.validTargetEntry.getBoolean(false);
    this.yaw = this.yawEntry.getDouble(-1);
    this.pitch = this.pitchEntry.getDouble(-1);
    this.latency = this.latencyEntry.getDouble(-1);
    double defaultPose[] = new double[] { -1, -1, -1 };
    this.poseX = this.targetPoseEntry.getDoubleArray(defaultPose)[0];
    this.poseY = this.targetPoseEntry.getDoubleArray(defaultPose)[1];
    this.poseAngle = this.targetPoseEntry.getDoubleArray(defaultPose)[2];

    double distanceLineEqM = 3.5729109;
    double distanceLineEqB = -4.02731519;
    this.distance = this.poseX < 0 ? -1 : (distanceLineEqM * this.poseX) + distanceLineEqB;
    this.turretLastTargetEntry.setDouble(this.lastTargetPos);
    this.turretLastTargetOffsetEntry.setDouble(this.turretOffset);
  }

  public void resetTurretEncoder() {
    this.turretEncoder.setPosition(0);
  }

  private double clamp(double val) {
    if(val > 180) {
      val -= 360;
    } else if(val < -180) {
      val += 360;
    }
    return val;
  }
}
