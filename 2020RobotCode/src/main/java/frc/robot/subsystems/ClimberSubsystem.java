/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final Solenoid climberLift;
  private final Solenoid climberRelease;
  private final Solenoid climberLock;
  private final Spark climbMotor;

  public ClimberSubsystem() {
    this.climberLift = new Solenoid(Constants.climberLift);
    this.climberRelease = new Solenoid(Constants.climberRelease);
    this.climberLock = new Solenoid(Constants.climberLock);
    this.climbMotor = new Spark(Constants.climberSpark);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void climberLock(){
    this.climberLock.set(false);
  }
  public void climberUnlock(){
    this.climberLock.set(true);
  }

  public void climberRelease() {
    this.climberRelease.set(true);
  }

  public void climb(double speed) {
    this.climbMotor.set(speed);
  }

  public void liftClimber() {
    this.climberLift.set(true);
  }

  public void lowerClimber() {
    this.climberLift.set(false);
  }

  public void stop() {
    this.climbMotor.set(0);
  } 
}
