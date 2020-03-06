/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

import frc.robot.Constants;
import frc.robot.utils.Logging;

public class IndexerSubsystem extends SubsystemBase {
  CANSparkMax indexingMotor;
  Spark intakeMotor;
  Solenoid intakeExtension;
  DigitalInput indexerFirstPos, indexerSecondPos, fullIndexerSensor;
  TurretSubsystem turretSubsystem;
  
  NetworkTableInstance table;
  NetworkTableEntry isFullEntry;
  NetworkTable pidTuningPVs;


  
  
  /**
   * Creates a new IndexerSubsystem.
   */
  
  public IndexerSubsystem(TurretSubsystem tSubsystem) {
    this.turretSubsystem = tSubsystem;
    this.indexingMotor = new CANSparkMax(Constants.indexingSparkMax, MotorType.kBrushless);
    this.indexingMotor.setSmartCurrentLimit(Constants.miniNeoSafeAmps);
    this.intakeMotor = new Spark(Constants.intakeSpark);
    this.intakeExtension = new Solenoid(Constants.intakeExtension);
    indexerFirstPos = new DigitalInput(Constants.indexerFirstPos);
    indexerSecondPos = new DigitalInput(Constants.indexerSecondPos);
    fullIndexerSensor = new DigitalInput(Constants.fullIndexerSensor);
    this.setupNetworkTables();
  }
  private void setupNetworkTables() {
    this.table = NetworkTableInstance.getDefault();
    this.pidTuningPVs = this.table.getTable("pidTuningPVs");
    this.isFullEntry = this.pidTuningPVs.getEntry("Is Indexer Full");
  }
  private void updateNetworkTables(){
    this.isFullEntry.setBoolean(this.isFull());
  }

  @Override
  public void periodic() {
    String status = String.format(
      "\nFirst sensor status: %s\nSecond sensor staus: %s\nFull sensor status: %s",
      ballAtIntake(),
      ballAtPosOne(),
      isFull());
    Logging.debug(status, "indexerStatus");
    this.updateNetworkTables();
  }

  public void intakeBall() {
    if(!isFull()) {
      intakeRaw(-Constants.intakeSpeed);
      if(ballAtIntake()) {
        indexerRaw(Constants.indexingSpeed);
        this.turretSubsystem.feedShooterRaw(-0.4);
      } else {
        indexerRaw(0);
      }
    } else {
      indexerRaw(0);
      intakeRaw(0);
    }
  }

  public void intakeRaw(double speed) {
    this.intakeMotor.set(speed);
  }

  public void indexerRaw(double speed) {
    this.indexingMotor.set(speed);
  }

  public void lowerIntake() {
    this.intakeExtension.set(true);
  }

  public void raiseIntake() {
    this.intakeExtension.set(false);
  }

  public void stop() {
    this.intakeMotor.set(0);
    this.indexingMotor.set(0);
  }

  public boolean isFull() {
    return !fullIndexerSensor.get();
  }

  public boolean ballAtIntake() {
    return !indexerFirstPos.get();
  }

  public boolean ballAtPosOne() {
    return !indexerSecondPos.get();
  }
}
