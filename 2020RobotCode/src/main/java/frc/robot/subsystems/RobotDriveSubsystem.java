/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class RobotDriveSubsystem extends SubsystemBase {
  /**
   * Creates a new RobotDriveSubsystem.
   */
  CANSparkMax 
    leftDrive,
    leftDriveB,
    rightDrive,
    rightDriveB; 
  CANPIDController
    leftPIDController,
    rightPIDController;
  CANEncoder
    leftEncoder, 
    rightEncoder;
  DifferentialDrive driveBase;

  public RobotDriveSubsystem() {
    //Set up motors
    this.leftDrive = new CANSparkMax(Constants.leftDrive, MotorType.kBrushless);
    this.leftDriveB = new CANSparkMax(Constants.leftDriveB, MotorType.kBrushless);
    this.rightDrive = new CANSparkMax(Constants.rightDrive, MotorType.kBrushless);
    this.rightDriveB = new CANSparkMax(Constants.rightDriveB, MotorType.kBrushless);
      //Set up follow
    this.leftDriveB.follow(this.leftDrive);
    this.rightDriveB.follow(this.rightDrive);
      //Set up DifferentialDrive
    this.driveBase = new DifferentialDrive(leftDrive, rightDrive);
      //Set up encoders
    this.leftEncoder = this.leftDrive.getEncoder();
    this.rightEncoder = this.rightDrive.getEncoder();
      //Set up PIDControlers
    this.leftPIDController = this.leftDrive.getPIDController();
    this.rightPIDController = this.rightDrive.getPIDController();
        //Set up PID values
    this.leftPIDController.setP(Constants.drivekP);
    this.leftPIDController.setI(Constants.drivekI);
    this.leftPIDController.setD(Constants.drivekD);
    this.leftPIDController.setIZone(Constants.drivekIz);
    this.leftPIDController.setFF(Constants.drivekFF);
    this.leftPIDController.setOutputRange(Constants.drivePIDMin,Constants.drivePIDMax);
    
    this.rightPIDController.setP(Constants.drivekP);
    this.rightPIDController.setI(Constants.drivekI);
    this.rightPIDController.setD(Constants.drivekD);
    this.rightPIDController.setIZone(Constants.drivekIz);
    this.rightPIDController.setFF(Constants.drivekFF);
    this.rightPIDController.setOutputRange(Constants.drivePIDMin,Constants.drivePIDMax);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    this.driveBase.tankDrive(leftSpeed, rightSpeed);
  }
  public void arcadeDrive(double xSpeed, double zRotation){
    this.driveBase.arcadeDrive(xSpeed, zRotation);
  }
  public void rawDriveToEncoder(double rotations){
    this.leftPIDController.setReference(rotations, ControlType.kPosition);
    this.rightPIDController.setReference(rotations, ControlType.kPosition);
  }


}
