/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.logging;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import io.github.pseudoresonance.pixy2api.links.SPILink;

public class VisionSubsystem extends SubsystemBase {
  /**
   * Creates a new VisionSubsystem.
   */
  
  Pixy2 pixy;
  Pixy2CCC pixyCCC;
  Block ball = null;
  NetworkTableInstance tableInstance;
  NetworkTable table;
  NetworkTableEntry validBall, ballX, ballY, ballWidth, ballHeight, ballArea, ballAge;


  public VisionSubsystem() {
    this.pixy = Pixy2.createInstance(new SPILink());
    this.pixy.init();
    this.tableInstance = NetworkTableInstance.getDefault();
    this.table = tableInstance.getTable("PixyVision");
    this.ballX = table.getEntry("ballX");
    this.ballY = table.getEntry("ballY");
    this.ballWidth = table.getEntry("ballWidth");
    this.ballHeight = table.getEntry("ballHeight");
    this.ballArea = table.getEntry("ballArea");
    this.ballAge = table.getEntry("ballAge");
    this.pixyCCC = pixy.getCCC();
  }

  boolean prevOn = true;
  public void setPixyLamp(boolean on) {
    if(on && !this.prevOn) {
      this.pixy.setLamp((byte) 1, (byte) 1);
    } else if (!on && this.prevOn){
      this.pixy.setLamp((byte) 0, (byte) 0);
    } else {

    }
  }

  public double getBallX() {
    if(this.ball == null) {
      return -1000;
    }
    //TODO magic numbers
    return this.ball.getX() - (316/2);
  }

  public double getBallY() {
    if(this.ball == null) {
      return -1;
    }
    return this.ball.getY();
  }

  public double getBallWidth() {
    if(this.ball == null) {
      return -1;
    }
    return this.ball.getWidth();
  }

  public double getBallHeight() {
    if(this.ball == null) {
      return -1;
    }
    return this.ball.getHeight();
  }

  public double getBallArea() {
    if(this.ball == null) {
      return -1;
    }
    return this.ball.getWidth() * this.ball.getHeight();
  }

  public double getBallAge() {
    if(this.ball == null) {
      return -1;
    }
    return this.ball.getAge();
  }

  public Block getLargestBall() {
    int blockCount = pixyCCC.getBlocks(false, Pixy2CCC.CCC_SIG1, 25);
    if (blockCount <= 0) {
      this.ball = null;
      return null;
    }
    ArrayList<Block> blocks = pixyCCC.getBlocks();
    logging.debug("arraySize: "+blockCount, "pixycam");
    Block largestBlock = null;
    for(Block block : blocks) {
      if(largestBlock == null) {
        largestBlock = block;
      } else if(largestBlock.getWidth() < block.getWidth()) {
        largestBlock = block;
      }
    }
    this.ball = largestBlock;
    return largestBlock;
  }

  public void sendDataToShuffleboard() {
    //getLargestBall();
    this.ballX.setDouble(getBallX());
    this.ballY.setDouble(getBallY());
    this.ballWidth.setDouble(getBallWidth());
    this.ballHeight.setDouble(getBallHeight());
    this.ballArea.setDouble(getBallArea());
    this.ballAge.setDouble(getBallAge());
  }

  public void reset() {
    setPixyLamp(false);
  }

  @Override
  public void periodic() {
    this.getLargestBall();
    sendDataToShuffleboard();
  }
}
