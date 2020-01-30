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
import com.revrobotics.CANPIDController;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.utils.Features;
import frc.robot.utils.logging;
import frc.robot.utils.MovingAverage;

public class RobotDriveSubsystem extends SubsystemBase {
  public TalonSRX leftTalon, rightTalon;
  VictorSPX leftVictor, rightVictor;
  CANPIDController pid;
  public AHRS gyro;
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
    this.pidTuningPVs = this.table.getTable("pidTuningPVs");

    this.gyroAngle = this.pidTuningPVs.getEntry("Angle");
    this.time = this.pidTuningPVs.getEntry("Time");
    this.PIDInfo = this.table.getTable("PID");
    this.camInfo = this.table.getTable("chameleon-vision").getSubTable("Shooter Targeting");
    this.targetPose = this.camInfo.getEntry("targetPose");
    this.distanceEntry = this.PIDInfo.getEntry("Distance");

    this.gyroAngle.setDouble(0);
    this.time.setDouble(0);



    if (Features.Shooter) {
    }
    this.leftTalon = new TalonSRX(Constants.leftDrive);
    this.leftVictor = new VictorSPX(Constants.leftDriveB);
    this.rightTalon = new TalonSRX(Constants.rightDrive);
    this.rightVictor = new VictorSPX(Constants.rightDriveB);
    

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

    this.gyroAngle.setDouble(angle);
    this.time.setDouble(System.currentTimeMillis());
    double poseX = this.targetPose.getDoubleArray(new double[]{0, 0})[0];
    double distance = (3.349506 * poseX) - 2.2405898;
    this.distanceToTarget.push(distance);
    this.distanceEntry.setDouble(this.distanceToTarget.getAverage());
  }

  
}
