/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
  CANSparkMax indexingMotor;
  Spark intakeMotor;

  /**
   * Creates a new IndexerSubsystem.
   */
  public IndexerSubsystem() {
    indexingMotor = new CANSparkMax(Constants.indexingSparkMax, MotorType.kBrushless);
    intakeMotor = new Spark(Constants.intakeSpark);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void intakeBall() {
    
  }

  public void intakeRaw(double speed) {
    intakeMotor.set(speed);
  }

  public void indexerRaw(double speed) {
    indexingMotor.set(speed);
  }
}
