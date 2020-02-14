/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

// import java.io.IOException;
// import java.nio.file.Path;
// import java.util.List;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.controller.RamseteController;
// import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  public final RobotDriveSubsystem driveSubsystem = new RobotDriveSubsystem();
  public final TurretSubsystem turretSubsystem = new TurretSubsystem(driveSubsystem);
  public final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

  // public final TestDriveSubsystem testDriveSubsystem = new
  // TestDriveSubsystem();

  public final RobotDriveCommand driveCommand = new RobotDriveCommand(driveSubsystem);
  public final TurretCommand turretCommand = new TurretCommand(turretSubsystem, driveSubsystem);
  public final IntakeCommand intakeCommand = new IntakeCommand(indexerSubsystem);

  // public final TestDriveCommand testDriveCommand = new
  // TestDriveCommand(testDriveSubsystem);

  public static Joystick driverLeft = new Joystick(0);
  public static Joystick driverRight = new Joystick(1);
  public static Joystick operator = new Joystick(2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set default commands
    setDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  private void setDefaultCommands() {

    this.driveSubsystem.setDefaultCommand(this.driveCommand);
    this.turretSubsystem.setDefaultCommand(this.turretCommand);
    this.indexerSubsystem.setDefaultCommand(this.intakeCommand);

    // this.testDriveSubsystem.setDefaultCommand(this.testDriveCommand);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    // var autoVoltageConstraint =
    // new DifferentialDriveVoltageConstraint(
    // new SimpleMotorFeedforward(Constants.ksVolts,
    // Constants.kvVoltSecondsPerMeter,
    // Constants.kaVoltSecondsSquaredPerMeter),
    // Constants.kDriveKinematics,
    // 10);

    // // Create config for trajectory
    // TrajectoryConfig config =
    // new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
    // Constants.kMaxAccelerationMetersPerSecondSquared)
    // // Add kinematics to ensure max speed is actually obeyed
    // .setKinematics(Constants.kDriveKinematics)
    // // Apply the voltage constraint
    // .addConstraint(autoVoltageConstraint);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // // Start at the origin facing the +X direction
    // new Pose2d(0, 0, new Rotation2d(0)),
    // // Pass through these two interior waypoints, making an 's' curve path
    // List.of(
    // new Translation2d(1, 1),
    // new Translation2d(2, -1)
    // ),
    // // End 3 meters straight ahead of where we started, facing forward
    // new Pose2d(3, 0, new Rotation2d(0)),
    // // Pass config
    // config
    // );

    // RamseteCommand ramseteCommand = new RamseteCommand(
    // exampleTrajectory,
    // this.testDriveSubsystem::getPose,
    // new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
    // new SimpleMotorFeedforward(Constants.ksVolts,
    // Constants.kvVoltSecondsPerMeter,
    // Constants.kaVoltSecondsSquaredPerMeter),
    // Constants.kDriveKinematics,
    // this.testDriveSubsystem::getWheelSpeeds,
    // new PIDController(Constants.kPDriveVel, 0, 0),
    // new PIDController(Constants.kPDriveVel, 0, 0),
    // // RamseteCommand passes volts to the callback
    // this.testDriveSubsystem::tankDriveVolts,
    // this.testDriveSubsystem
    // );

    // // Run path following command, then stop at the end.
    // return ramseteCommand.andThen(() -> this.testDriveSubsystem.tankDriveVolts(0,
    // 0));

    return null;
  }
}
