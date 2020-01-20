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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Utilities;
import frc.robot.utils.PID;
import frc.robot.utils.logging;

public class RobotDriveSubsystem extends SubsystemBase {
  public TalonSRX leftTalon;
  public TalonSRX rightTalon;
  VictorSPX leftVictor, rightVictor;
  CANSparkMax testMotor, testMotor2;
  CANPIDController pid;
  CANEncoder encoder;
  public AHRS gyro;
  NetworkTable gyroInfo;
  public NetworkTableEntry gyroAngle, time;
  PID testMotorPID;

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

    this.gyroInfo = NetworkTableInstance.getDefault().getTable("gyroInfo");

    this.gyroAngle = this.gyroInfo.getEntry("Angle");
    this.gyroAngle.setValue(0);
    this.time = this.gyroInfo.getEntry("Time");
    this.time.setValue(0);

    this.testMotor = new CANSparkMax(Constants.testSparkMax, MotorType.kBrushless);
    this.testMotor2 = new CANSparkMax(Constants.testSparkMax2, MotorType.kBrushless);
    this.leftTalon = new TalonSRX(Constants.leftDrive);
    this.leftVictor = new VictorSPX(Constants.leftDriveB);
    this.rightTalon = new TalonSRX(Constants.rightDrive);
    this.rightVictor = new VictorSPX(Constants.rightDriveB);
    
    this.encoder = this.testMotor2.getEncoder();
    //like our magic numbers?
    //this.testMotorPID = new PID(0, 0.0006, 0.002465, 3.65156E-05, 0);
    this.testMotorPID = new PID(0,0.0006,0.00212,0.00004245,0.000170949,0.004775503);

    leftTalon.setInverted(false);
    leftTalon.setSensorPhase(false);
    leftVictor.follow(leftTalon);
    leftVictor.setInverted(InvertType.FollowMaster);

    rightTalon.setInverted(true);
    rightTalon.setSensorPhase(false);
    rightVictor.follow(rightTalon);
    rightVictor.setInverted(InvertType.FollowMaster);

  }

  @Override
  public void periodic() {
    updateGyro();
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

  public void updateGyro() {
    double angle = this.gyro.getAngle();
    logging.info("Gyro angle:"+angle, "gyro");
    logging.debug("Time:"+System.currentTimeMillis(), "time");
    if (angle > 360) {
      angle -= 360;
    } else if (angle < 0) {
      angle += 360;
    }
    this.gyroAngle.setValue(angle);
    this.time.setValue(System.currentTimeMillis());
  }

  public void testMotor(final double speed) {
    // Target velocity: 4851
    // Max velocity: 5700
    final double targetVel = speed * 5415;
    this.testMotorPID.updateSetpoint(targetVel);
    double workingSpeed = this.testMotorPID.run(this.encoder.getVelocity(), Robot.iterTime);
    if (speed == 0){
      workingSpeed = 0.0;
    }

    if(workingSpeed > 0) {
      workingSpeed = 0;
    }

    workingSpeed = speed; //Bypass PID if it's going nuts
    testMotor.set(-workingSpeed);
    testMotor2.set(workingSpeed);
    //System.out.println("Input:"+workingSpeed+" Shooter:"+this.encoder.getVelocity()+" Target:"+targetVel);
    //System.out.println("Shooter speed: "+this.encoder.getVelocity());
    //Utilities.logging("Shooter speed: "+this.encoder.getVelocity(), "DEBUG");
    logging.debug("Input:"+workingSpeed,"shooterPID");
    logging.debug("Shooter:"+this.encoder.getVelocity(),"shooterPID");
    logging.debug("Target:"+targetVel,"shooterPID");
  }

  public void resetShooterIntegral() {
    this.testMotorPID.resetIntegral();
  }
}
