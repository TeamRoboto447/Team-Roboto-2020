/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.lang.Math;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

import frc.robot.utils.EdgeDetector;
import frc.robot.Utilities;

public class RobotDriveSubsystem extends SubsystemBase {
  /**
   * Creates a new RobotDriveSubsystem.
   */
  CANSparkMax leftDrive, leftDriveB, rightDrive, rightDriveB;
  CANPIDController leftPIDController, rightPIDController;
  CANEncoder leftEncoder, rightEncoder;
  DifferentialDrive driveBase;
  boolean driveInverted;

  boolean drivingTo;
  boolean firstDrivingTo;
  EdgeDetector stopDriveTo;

  AHRS gyro;

  public RobotDriveSubsystem() {
    this.gyro = new AHRS(SPI.Port.kMXP);
    this.drivingTo=false;
    this.firstDrivingTo=true;
    // Set up motors
    this.leftDrive = new CANSparkMax(Constants.leftDrive, MotorType.kBrushless);
    this.leftDriveB = new CANSparkMax(Constants.leftDriveB, MotorType.kBrushless);
    this.rightDrive = new CANSparkMax(Constants.rightDrive, MotorType.kBrushless);
    this.rightDriveB = new CANSparkMax(Constants.rightDriveB, MotorType.kBrushless);

    this.setInvertedDrive(false);
    // Set up follow
    this.leftDriveB.follow(this.leftDrive);
    this.rightDriveB.follow(this.rightDrive);
    // Set up DifferentialDrive
    // this.driveBase = new DifferentialDrive(leftDrive, rightDrive);
    // Set up encoders
    this.leftEncoder = this.leftDrive.getEncoder();
    this.rightEncoder = this.rightDrive.getEncoder();
    // Set up PIDControlers
    this.leftPIDController = this.leftDrive.getPIDController();
    this.rightPIDController = this.rightDrive.getPIDController();
    // Set up PID values
    this.leftPIDController.setP(Constants.drivekP);
    this.leftPIDController.setI(Constants.drivekI);
    this.leftPIDController.setD(Constants.drivekD);
    this.leftPIDController.setIZone(Constants.drivekIz);
    this.leftPIDController.setFF(Constants.drivekFF);
    this.leftPIDController.setOutputRange(Constants.drivePIDMin, Constants.drivePIDMax);

    this.rightPIDController.setP(Constants.drivekP);
    this.rightPIDController.setI(Constants.drivekI);
    this.rightPIDController.setD(Constants.drivekD);
    this.rightPIDController.setIZone(Constants.drivekIz);
    this.rightPIDController.setFF(Constants.drivekFF);
    this.rightPIDController.setOutputRange(Constants.drivePIDMin, Constants.drivePIDMax);

    this.leftEncoder.setPosition(0);
    this.rightEncoder.setPosition(0);

    this.stopDriveTo = new EdgeDetector(true);
  }

  @Override
  public void periodic() {
    if (this.stopDriveTo.detect(this.drivingTo)){
      this.firstDrivingTo = true;
    }
    this.drivingTo = false;
  }

  public void setInvertedDrive(boolean invert) {
    this.driveInverted = invert;
    this.rightDrive.setInverted(!this.driveInverted);
    this.leftDrive.setInverted(this.driveInverted);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    double deadzone = 0.1;
    leftSpeed = Utilities.adjustForDeadzone(leftSpeed, deadzone);
    rightSpeed = Utilities.adjustForDeadzone(rightSpeed, deadzone);
    
    this.setRelativeLeft(leftSpeed);
    this.setRelativeRight(rightSpeed);
  }

  public void arcadeDrive(double xSpeed, double zRotation) {
    double lSpeed = xSpeed - zRotation;
    double rSpeed = xSpeed + zRotation;
    this.tankDrive(lSpeed, rSpeed);
  }

  public void rawDriveToEncoder(double rotations) {
    this.drivingTo = true;
    if (this.firstDrivingTo){
      this.resetEncoders();
      this.firstDrivingTo = false;
    }
    this.leftPIDController.setReference(rotations, ControlType.kPosition);
    this.rightPIDController.setReference(rotations, ControlType.kPosition);
  }
  public void driveToEncoder(double rotations) {
    double encRotations = rotations * Constants.lowGearRatio;
    this.rawDriveToEncoder(encRotations);
  }
  public void driveToInches(double inches){
    double rotations = inches / (Constants.wheelDiameter*Math.PI);
    this.driveToEncoder(rotations);
  }
  public void driveToFeet(double feet){
    double inches = feet * 12;
    this.driveToInches(inches);
  }

  private void setRelativeLeft(double speed) {
    if (this.driveInverted) {
      this.rightDrive.set(speed);
    } else {
      this.leftDrive.set(speed);
    }
  }

  private void setRelativeRight(double speed) {
    if (this.driveInverted) {
      this.leftDrive.set(speed);
    } else {
      this.rightDrive.set(speed);
    }
  }

  private void resetEncoders(){
    this.rightEncoder.setPosition(0);
    this.leftEncoder.setPosition(0);
  }

  public double getAngle() {
    return gyro.getAngle();
  }

  public void resetAngle() {
    this.gyro.reset();
  }
}
