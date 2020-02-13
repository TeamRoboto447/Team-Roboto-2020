/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

public class ControlPanelSubsystem extends SubsystemBase {

  I2C.Port i2cPort = I2C.Port.kOnboard;
  ColorSensorV3 colorSensor;
  ColorMatch colorMatcher;
  Color blueTarget, redTarget, greenTarget, yellowTarget;

  NetworkTable colorSensorTable;
  NetworkTableInstance table;
  NetworkTableEntry colorR, colorB, colorG, colorName;


  public ControlPanelSubsystem() {
    // Set up network table.
    this.colorSensor = new ColorSensorV3(i2cPort);
    this.table = NetworkTableInstance.getDefault();
    this.colorSensorTable = this.table.getTable("colorSensor");
    this.colorR = this.colorSensorTable.getEntry("Color R Value");
    this.colorG = this.colorSensorTable.getEntry("Color G Value");
    this.colorB = this.colorSensorTable.getEntry("Color B Value");
    this.colorName = this.colorSensorTable.getEntry("Color Name");

    // Set initial NT values.
    this.colorR.setNumber(0);
    this.colorG.setNumber(0);
    this.colorB.setNumber(0);
    this.colorName.setString("");

    // Create known colors.
    //TODO Magic Numbers
    this.blueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    this.greenTarget = ColorMatch.makeColor(0.192, 0.561, 0.240);
    this.redTarget = ColorMatch.makeColor(0.62, 0.302, 0.069);
    this.yellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    // Create color matcher
    this.colorMatcher = new ColorMatch();

    // Add known colors.
    this.colorMatcher.addColorMatch(this.blueTarget);
    this.colorMatcher.addColorMatch(this.greenTarget);
    this.colorMatcher.addColorMatch(this.redTarget);
    this.colorMatcher.addColorMatch(this.yellowTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateNetworkTable();
  }

  private Color getColor() {
    Color detectedColor = colorSensor.getColor();
    return detectedColor;
  }

  public double[] getRGB() {
    double[] rgbVals = new double[]{0, 0, 0};
    Color color = getColor();
    
    rgbVals[0] = color.red;
    rgbVals[1] = color.green;
    rgbVals[2] = color.blue;

    return rgbVals;
  }

  public String getColorName() {
    String colorName = "";
    Color color = getColor();
    ColorMatchResult match = colorMatcher.matchClosestColor(color);
    
    if(match == null) return "Unknown";

    if(match.color == this.blueTarget) {
      colorName = "Blue";
    } else if(match.color == this.redTarget) {
      colorName = "Red";
    } else if(match.color == this.greenTarget) {
      colorName = "Green";
    } else if(match.color == this.yellowTarget) {
      colorName = "Yellow";
    } else {
      colorName = "Unknown";
    }

    return colorName;
  }

  private void updateNetworkTable() {
    double[] rgbVals = getRGB();
    this.colorR.setNumber(rgbVals[0]);
    this.colorG.setNumber(rgbVals[1]);
    this.colorB.setNumber(rgbVals[2]);
    this.colorName.setString(getColorName());
  }
}
