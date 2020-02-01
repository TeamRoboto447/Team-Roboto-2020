/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utils.Features;
import frc.robot.utils.PID;
import frc.robot.utils.logging;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import java.lang.Math;

public class TurretSubsystem extends SubsystemBase {
  /**
   * Creates a new TurretSubsystem.
   */
  NetworkTableInstance table;
  NetworkTable PIDInfo, pidTuningPVs, camInfo;
  NetworkTableEntry shootPEntry, shootIEntry, shootDEntry, shootFFmEntry, shootFFbEntry,
    bypassShooterPIDEntry, distanceEntry, shooterSpeedEntry, shooterCurrSpeedEntry;
  PID shootingMotorPID;
  Boolean bypassShooterPID;
  Double shootP, shootI, shootD, shootFFm, shootFFb;
  
  CANSparkMax shootingMotor, shootingMotor2;
  CANEncoder encoder;
  public TurretSubsystem() {
    // Set up Network Tables
    this.table = NetworkTableInstance.getDefault();
    this.PIDInfo = this.table.getTable("PID");
    this.pidTuningPVs = this.table.getTable("pidTuningPVs");
    this.camInfo = this.table.getTable("chameleon-vision").getSubTable("Shooter Targeting");
    
    this.shootPEntry = this.PIDInfo.getEntry("shootkP");
    this.shootIEntry = this.PIDInfo.getEntry("shootkI");
    this.shootDEntry = this.PIDInfo.getEntry("shootkD");
    this.shootFFmEntry = this.PIDInfo.getEntry("shootkFFm");
    this.shootFFbEntry = this.PIDInfo.getEntry("shootkFFb");
    this.bypassShooterPIDEntry = this.PIDInfo.getEntry("bypassShooterPID");
    this.shooterSpeedEntry = this.PIDInfo.getEntry("shootTargetSpeed");
    this.shooterCurrSpeedEntry = this.pidTuningPVs.getEntry("Shooter Speed");

    this.shooterCurrSpeedEntry.setDouble(0);



    // Set up Motors
    this.shootingMotor = new CANSparkMax(Constants.shooterSparkMax, MotorType.kBrushless);
    this.shootingMotor2 = new CANSparkMax(Constants.shooterSparkMax2, MotorType.kBrushless);
    if (Features.Shooter){
      this.encoder = this.shootingMotor.getEncoder();
    }
    
    //like our magic numbers?
    //this.testMotorPID = new PID(0, 0.0006, 0.002465, 3.65156E-05, 0);
    this.shootingMotorPID = new PID(
      0, // Default setpoint
      Constants.shooterkP, // 
      Constants.shooterkI, //
      Constants.shooterkD, //
      Constants.shooterkFFm, //
      Constants.shooterkFFb, //
      -100, // Minimum integral
      100); // Maximum integral
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateShooterPIDValues();
    updateNetworkTables();
  }


  double prevSetpoint = 0;
  public void runShooterAtSpeed(double speed) {
    if (!Features.Shooter) {
      return;
    }
    
    // Target velocity: 4851
    // Max velocity: 5700
    double targetVel = speed * 5415;
    this.shooterSpeedEntry.setDouble(targetVel);
    this.shootingMotorPID.updateSetpoint(targetVel);
    
    double delta = this.prevSetpoint - targetVel;
    if(delta > 100 || delta < -100) {
      this.shootingMotorPID.resetIntegral();
    }
    
    double workingSpeed = this.shootingMotorPID.run(this.encoder.getVelocity());
    if(workingSpeed < 0) {
      workingSpeed = 0;
    }

    if(speed <= 0){
      workingSpeed = speed;
    }
    if(this.bypassShooterPID) {
      workingSpeed = speed; //Bypass PID if it's going nuts
    }
    
    shootingMotor.set(workingSpeed);
    shootingMotor2.set(-workingSpeed);
    //System.out.println("Input:"+workingSpeed+" Shooter:"+this.encoder.getVelocity()+" Target:"+targetVel);
    //System.out.println("Shooter speed: "+this.encoder.getVelocity());
    //Utilities.logging("Shooter speed: "+this.encoder.getVelocity(), "DEBUG");

    logging.debug("FF set: " + targetVel + ", FF out: " +this.encoder.getVelocity(),"FFTuning");

    logging.debug("Input:"+workingSpeed,"shooterPID");
    logging.debug("Shooter:"+this.encoder.getVelocity(),"shooterPID");
    logging.debug("Target:"+targetVel,"shooterPID");
    this.prevSetpoint = targetVel;
  }

  public void resetShooterIntegral() {
    this.shootingMotorPID.resetIntegral();
  }

  public void updateNetworkTables() {
    this.shooterCurrSpeedEntry.setDouble(this.encoder.getVelocity());
  }

  public void updateShooterPIDValues() {
    this.shootP = this.shootPEntry.getDouble(Constants.shooterkP);
    this.shootI = this.shootIEntry.getDouble(Constants.shooterkI);
    this.shootD = this.shootDEntry.getDouble(Constants.shooterkD);
    this.shootFFm = this.shootFFmEntry.getDouble(Constants.shooterkFFb);
    this.shootFFb = this.shootFFbEntry.getDouble(Constants.shooterkFFm);
    this.bypassShooterPID = this.bypassShooterPIDEntry.getBoolean(Constants.bypassShooterPID);

    logging.debug("Shooter PID Values: kP: "+ this.shootP +" kI: "+ this.shootI + " kD: "+this.shootD, "shootingPID");

    shootingMotorPID.updateP(this.shootP);
    shootingMotorPID.updateI(this.shootI);
    shootingMotorPID.updateD(this.shootD);
    shootingMotorPID.updateFF(this.shootFFm, this.shootFFb);
  }
}
