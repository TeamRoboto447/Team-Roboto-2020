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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities;

public class RobotDriveSubsystem extends SubsystemBase {
  public TalonSRX leftTalon;
public TalonSRX rightTalon;
  VictorSPX leftVictor, rightVictor;
  CANSparkMax testMotor;
  public Gyro gyro;
  NetworkTable gyroInfo;
  public NetworkTableEntry gyroAngle, time;


  private boolean driveInverted = false;
  public boolean getDriveInverted() {
    return this.driveInverted;
  }
  
  public void setDriveInverted(final boolean value) {
    this.driveInverted = value;
  }

  public RobotDriveSubsystem() {
    this.gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
    this.gyro.calibrate();

    this.gyroInfo = NetworkTableInstance.getDefault().getTable("gyroInfo");

    this.gyroAngle = this.gyroInfo.getEntry("Angle");
    this.gyroAngle.setValue(0);
    this.time = this.gyroInfo.getEntry("Time");
    this.time.setValue(0);

    testMotor = new CANSparkMax(Constants.testSparkMax, MotorType.kBrushless);
    leftTalon = new TalonSRX(Constants.leftDrive);
    leftVictor = new VictorSPX(Constants.leftDriveB);
    rightTalon = new TalonSRX(Constants.rightDrive);
    rightVictor = new VictorSPX(Constants.rightDriveB);

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

  public void tankDrive(double leftSpeed, double rightSpeed, boolean driveInverted) {
    if(driveInverted) {
      leftTalon.set(ControlMode.PercentOutput, -rightSpeed);
      rightTalon.set(ControlMode.PercentOutput, -leftSpeed);
    } else {
      leftTalon.set(ControlMode.PercentOutput, leftSpeed);
      rightTalon.set(ControlMode.PercentOutput, rightSpeed);
    }
  }

  public void driveToInch(double targetLeft, double targetRight) {
    leftTalon.set(ControlMode.MotionMagic, Utilities.encoderToInch(targetLeft));
    rightTalon.set(ControlMode.MotionMagic, Utilities.encoderToInch(targetRight));
  }

  public void driveToEncode(double targetLeft, double targetRight) {
    leftTalon.set(ControlMode.MotionMagic, targetLeft);
    rightTalon.set(ControlMode.MotionMagic, targetRight);
  }

  public void turnToZeroVeryInnacurate() {
    double speed = 0.75;
    double angle = gyroAngle.getDouble(0);
    Utilities.logging(angle+"", "DEBUG");
    if(angle >= 0 && angle <= 180) {
      this.tankDrive(speed, -speed, false);
    } else if(angle <= 359 && angle >= 181) {
      this.tankDrive(-speed, speed, false);
    }
  }

  public void updateGyro() {
    double angle = this.gyro.getAngle();
    if(angle > 360) {
      angle -= 360;
    } else if(angle < 0) {
      angle += 360;
    }
    this.gyroAngle.setValue(angle);
    this.time.setValue(System.currentTimeMillis());
  }

  public void testMotor(double speed) {
    testMotor.set(speed);
  }
}
