/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlingSubsystem extends SubsystemBase {
  NetworkTable bling;
  NetworkTableEntry blingSelect;
  NetworkTableInstance table;

  public BlingSubsystem() {
    setupNetworkTables();
  }

  @Override
  public void periodic() {

  }

  private void setupNetworkTables() {
    this.table = NetworkTableInstance.getDefault();
    this.bling = this.table.getTable("Bling");
    this.blingSelect = this.bling.getEntry("blingSelect");
    this.blingSelect.setString("idle");
  }

  public void setBling() {
    
  }
}
