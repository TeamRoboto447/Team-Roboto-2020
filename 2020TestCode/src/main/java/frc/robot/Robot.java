/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.*;
import edu.wpi.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autoCommand;
  private RobotContainer robot;
  private UsbCamera camera0, camera1;
  private VideoSink camServer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  NetworkTable PIDTune;
  NetworkTableInstance table;
  NetworkTableEntry P, I, D, driveP, driveI, driveD;
  
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robot = new RobotContainer();
    camera0 = CameraServer.getInstance().startAutomaticCapture(0);
    camera0.setResolution(160, 120);
    camera0.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    camera1 = CameraServer.getInstance().startAutomaticCapture(1);
    camera1.setResolution(160, 120);
    camera1.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    camServer = CameraServer.getInstance().getServer();
    
    this.table = NetworkTableInstance.getDefault();

    this.PIDTune = this.table.getTable("chameleon-vision").getSubTable("PID");
    this.P = this.PIDTune.getEntry("P");
    this.I = this.PIDTune.getEntry("I");
    this.D = this.PIDTune.getEntry("D");
    this.P.setDouble(0.0144);
    this.I.setDouble(0.0259459);
    this.D.setDouble(0.001998);

    this.driveP = this.PIDTune.getEntry("driveP");
    this.driveI = this.PIDTune.getEntry("driveI");
    this.driveD = this.PIDTune.getEntry("driveD");
    this.driveP.setDouble(0.05);
    this.driveI.setDouble(0);
    this.driveD.setDouble(0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    autoCommand = robot.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autoCommand != null) {
      autoCommand.cancel();
    }

  }


  private boolean lookForward = true;
  public boolean getLookForward() {
    return this.lookForward;
  }
  
  private boolean beingPressed = false;

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    // If pressed and wasn't pressed on the last cycle
    // then flip the boolean value 
    // then set the is being pressed state
    // then set the camera to the inverse camera

    // If not being pressed, set the being pressed state

    if(RobotContainer.driverRight.getRawButton(1)) {
      if(!this.beingPressed) {
        this.lookForward = !this.lookForward;
        this.beingPressed = true;
        if(this.lookForward) {
          camServer.setSource(camera1);
        } else {
          camServer.setSource(camera0);
        }
        this.robot.driveSubsystem.setDriveInverted(!this.lookForward);
      }
    } else {
      this.beingPressed = false;
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
