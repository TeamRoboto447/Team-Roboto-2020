/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

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
  public final IndexerSubsystem indexerSubsystem = new IndexerSubsystem(turretSubsystem);

  public final RobotDriveCommand driveCommand = new RobotDriveCommand(driveSubsystem);
  public final TurretCommand turretCommand = new TurretCommand(turretSubsystem);
  public final IntakeCommand intakeCommand = new IntakeCommand(indexerSubsystem, turretSubsystem);

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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    SequentialCommandGroup threeBallAuto = new SequentialCommandGroup(
      new DriveToPosition(this.driveSubsystem, Utilities.inchToEncoder(12)),
      new AimAndShoot(this.turretSubsystem, this.indexerSubsystem, -5)
    );

    SequentialCommandGroup sixBallAuto = new SequentialCommandGroup(
      new AimAndShoot(this.turretSubsystem, this.indexerSubsystem, -5),
      new ParallelCommandGroup(
        new DriveToPosition(this.driveSubsystem, Utilities.feetToEncoder(-5)),
        new IntakeBalls(this.indexerSubsystem)
      ),
      new DriveToPosition(this.driveSubsystem, Utilities.feetToEncoder(5)),
      new AimAndShoot(this.turretSubsystem, this.indexerSubsystem, -5)
    );

    return threeBallAuto;
  }
}
