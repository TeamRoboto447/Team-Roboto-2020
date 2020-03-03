/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.utils.Features;
import frc.robot.utils.PID;
import frc.robot.utils.logging;
import frc.robot.utils.MovingAverage;

public class RobotDriveSubsystem extends SubsystemBase {
  public TalonSRX leftTalon, rightTalon;
  VictorSPX leftVictor, rightVictor;
  CANSparkMax shootingMotor, shootingMotor2;
  CANPIDController pid;
  CANEncoder encoder;
  public AHRS gyro;
  PID shootingMotorPID;
  Double shootP, shootI, shootD, shootFFm, shootFFb;
  Boolean bypassShooterPID;
  MovingAverage distanceToTarget;

  NetworkTable pidTuningPVs, PIDInfo, camInfo;
  NetworkTableInstance table;
  public NetworkTableEntry gyroAngle, shooterSpeed, time, targetPose,
    shootPEntry, shootIEntry, shootDEntry, shootFFmEntry, shootFFbEntry,
    distanceEntry, shooterSpeedEntry, bypassShooterPIDEntry;

  private boolean driveInverted = false;
  public boolean getDriveInverted() {
    return this.driveInverted;
  }
  
  public void setDriveInverted(final boolean value) {
    this.driveInverted = value;
  }

  public RobotDriveSubsystem() {
    this.gyro = new AHRS(SPI.Port.kMXP);
    this.gyro.reset();


    // Get all used NT entries
    this.table = NetworkTableInstance.getDefault();
    this.pidTuningPVs = NetworkTableInstance.getDefault().getTable("pidTuningPVs");

    this.shooterSpeed = this.pidTuningPVs.getEntry("Shooter Speed");
    this.gyroAngle = this.pidTuningPVs.getEntry("Angle");
    this.time = this.pidTuningPVs.getEntry("Time");
    this.PIDInfo = this.table.getTable("PID");
    this.camInfo = this.table.getTable("chameleon-vision").getSubTable("Shooter Targeting");
    this.targetPose = this.camInfo.getEntry("targetPose");
    
    this.shootPEntry = this.PIDInfo.getEntry("shootkP");
    this.shootIEntry = this.PIDInfo.getEntry("shootkI");
    this.shootDEntry = this.PIDInfo.getEntry("shootkD");
    this.shootFFmEntry = this.PIDInfo.getEntry("shootkFFm");
    this.shootFFbEntry = this.PIDInfo.getEntry("shootkFFb");
    this.bypassShooterPIDEntry = this.PIDInfo.getEntry("bypassShooterPID");
    this.distanceEntry = this.PIDInfo.getEntry("Distance");
    this.shooterSpeedEntry = this.PIDInfo.getEntry("shootTargetSpeed");

    this.shooterSpeed.setDouble(0);
    this.gyroAngle.setDouble(0);
    this.time.setDouble(0);


    if (Features.Shooter) {
      this.shootingMotor = new CANSparkMax(Constants.shooterSparkMax, MotorType.kBrushless);
      this.shootingMotor2 = new CANSparkMax(Constants.shooterSparkMax2, MotorType.kBrushless);
    }
    this.leftTalon = new TalonSRX(Constants.leftDrive);
    this.leftVictor = new VictorSPX(Constants.leftDriveB);
    this.rightTalon = new TalonSRX(Constants.rightDrive);
    this.rightVictor = new VictorSPX(Constants.rightDriveB);
    
    if (Features.Shooter){
      this.encoder = this.shootingMotor.getEncoder();
    }
    
    //like our magic numbers?
    //this.testMotorPID = new PID(0, 0.0006, 0.002465, 3.65156E-05, 0);
    this.shootingMotorPID = new PID(
      0, // Default setpoint
      Constants.shooterkP, // 
      Constants.shooterkI, //
      Constants.shooterkD, //
      Constants.shooterkFFm, //
      Constants.shooterkFFb, //
      -100, // Minimum integral
      100); // Maximum integral

    leftTalon.setInverted(false);
    leftTalon.setSensorPhase(false);
    leftVictor.follow(leftTalon);
    leftVictor.setInverted(InvertType.FollowMaster);

    rightTalon.setInverted(true);
    rightTalon.setSensorPhase(false);
    rightVictor.follow(rightTalon);
    rightVictor.setInverted(InvertType.FollowMaster);

    this.distanceToTarget = new MovingAverage(20);
  }

  @Override
  public void periodic() {
    updateNetworkTable();
    updateShooterPIDValues();
  }

  public void tankDrive(final double leftSpeed, final double rightSpeed, final boolean driveInverted) {
    if (driveInverted) {
      leftTalon.set(ControlMode.PercentOutput, -rightSpeed);
      rightTalon.set(ControlMode.PercentOutput, -leftSpeed);
    } else {
      leftTalon.set(ControlMode.PercentOutput, leftSpeed);
      rightTalon.set(ControlMode.PercentOutput, rightSpeed);
    }
  }

  public void driveToInch(final double targetLeft, final double targetRight) {
    leftTalon.set(ControlMode.MotionMagic, Utilities.encoderToInch(targetLeft));
    rightTalon.set(ControlMode.MotionMagic, Utilities.encoderToInch(targetRight));
  }

  public void driveToEncode(final double targetLeft, final double targetRight) {
    leftTalon.set(ControlMode.MotionMagic, targetLeft);
    rightTalon.set(ControlMode.MotionMagic, targetRight);
  }

  public void turnToZeroVeryInnacurate() {
    final double speed = 0.75;
    final double angle = gyroAngle.getDouble(0);
    if (angle >= 0 && angle <= 180) {
      this.tankDrive(speed, -speed, false);
    } else if (angle <= 359 && angle >= 181) {
      this.tankDrive(-speed, speed, false);
    }
  }

  public void updateNetworkTable() {
    double angle = this.gyro.getAngle();
    logging.info("Gyro angle:"+angle, "gyro");
    logging.debug("Time:"+System.currentTimeMillis(), "time");
    if (angle > 360) {
      angle -= 360;
    } else if (angle < 0) {
      angle += 360;
    }

    this.shooterSpeed.setDouble(this.encoder.getVelocity());
    this.gyroAngle.setDouble(angle);
    this.time.setDouble(System.currentTimeMillis());
    double poseX = this.targetPose.getDoubleArray(new double[]{0, 0})[0];
    double distance = (3.349506 * poseX) - 2.2405898;
    this.distanceToTarget.push(distance);
    this.distanceEntry.setDouble(this.distanceToTarget.getAverage());
  }

  double prevSetpoint = 0;
  public void runShooterAtSpeed(double speed) {
    if (!Features.Shooter) {
      return;
    }
    
    // Target velocity: 4851
    // Max velocity: 5700
    double targetVel = speed * 5415;
    this.shooterSpeedEntry.setDouble(targetVel);
    this.shootingMotorPID.updateSetpoint(targetVel);
    
    double delta = this.prevSetpoint - targetVel;
    if(delta > 100 || delta < -100) {
      this.shootingMotorPID.resetIntegral();
    }
    
    double workingSpeed = this.shootingMotorPID.run(this.encoder.getVelocity());
    if(workingSpeed < 0) {
      workingSpeed = 0;
    }

    if(speed <= 0){
      workingSpeed = speed;
    }
    if(this.bypassShooterPID) {
      workingSpeed = speed; //Bypass PID if it's going nuts
    }
    
    shootingMotor.set(workingSpeed);
    shootingMotor2.set(-workingSpeed);
    //System.out.println("Input:"+workingSpeed+" Shooter:"+this.encoder.getVelocity()+" Target:"+targetVel);
    //System.out.println("Shooter speed: "+this.encoder.getVelocity());
    //Utilities.logging("Shooter speed: "+this.encoder.getVelocity(), "DEBUG");

    logging.debug("FF set: " + targetVel + ", FF out: " +this.encoder.getVelocity(),"FFTuning");

    logging.debug("Input:"+workingSpeed,"shooterPID");
    logging.debug("Shooter:"+this.encoder.getVelocity(),"shooterPID");
    logging.debug("Target:"+targetVel,"shooterPID");
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

    logging.debug("Shooter PID Values: kP: "+ this.shootP +" kI: "+ this.shootI + " kD: "+this.shootD, "shootingPID");

    shootingMotorPID.updateP(this.shootP);
    shootingMotorPID.updateI(this.shootI);
    shootingMotorPID.updateD(this.shootD);
    shootingMotorPID.updateFF(this.shootFFm, this.shootFFb);
  }

  
}
