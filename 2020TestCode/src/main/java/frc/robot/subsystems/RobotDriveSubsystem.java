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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class RobotDriveSubsystem extends SubsystemBase {
  TalonSRX leftTalon, rightTalon;
  VictorSPX leftVictor, rightVictor;


  private boolean driveInverted = false;
  public boolean getDriveInverted() {
    return this.driveInverted;
  }
  public void setDriveInverted(boolean value) {
    this.driveInverted = value;
  }

  public RobotDriveSubsystem() {
    leftTalon = new TalonSRX(RobotMap.leftDrive);
    leftVictor = new VictorSPX(RobotMap.leftDriveB);
    rightTalon = new TalonSRX(RobotMap.rightDrive);
    rightVictor = new VictorSPX(RobotMap.rightDriveB);

    leftTalon.setInverted(true);
    leftVictor.follow(leftTalon);
    leftVictor.setInverted(InvertType.FollowMaster);

    rightTalon.setInverted(false);
    rightVictor.follow(rightTalon);
    leftVictor.setInverted(InvertType.FollowMaster);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
}
