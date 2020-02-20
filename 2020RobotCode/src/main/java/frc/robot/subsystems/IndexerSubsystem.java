/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
  CANSparkMax indexingMotor;
  Spark intakeMotor;
  Solenoid intakeExtension;

  /**
   * Creates a new IndexerSubsystem.
   */
  public IndexerSubsystem() {
    this.indexingMotor = new CANSparkMax(Constants.indexingSparkMax, MotorType.kBrushless);
    this.indexingMotor.setSmartCurrentLimit(Constants.miniNeoSafeAmps);
    this.intakeMotor = new Spark(Constants.intakeSpark);
    this.intakeExtension = new Solenoid(Constants.intakeExtension);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeBall() {
    
  }

  public void intakeRaw(double speed) {
    this.intakeMotor.set(speed);
  }

  public void indexerRaw(double speed) {
    this.indexingMotor.set(speed);
  }

  public void extendIntake(boolean extend) {
    this.intakeExtension.set(extend);
  }

  public void stop() {
    this.intakeMotor.set(0);
    this.indexingMotor.set(0);
  }
}
