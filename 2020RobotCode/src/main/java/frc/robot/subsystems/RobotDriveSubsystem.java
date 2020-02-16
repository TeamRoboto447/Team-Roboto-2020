package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;

import com.kauailabs.navx.frc.AHRS;
import com.opencsv.CSVWriter;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities;

public class RobotDriveSubsystem extends SubsystemBase {
  private final CANSparkMax leftDrive, leftDriveB, rightDrive, rightDriveB;

  // The motors on the left side of the drive.
  private final SpeedControllerGroup m_leftMotors;

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors;

  // The robot's drive
  private final DifferentialDrive m_drive;

  // The left-side drive encoder
  private final CANEncoder m_leftEncoder;

  // The right-side drive encoder
  private final CANEncoder m_rightEncoder;

  // The gyro sensor
  private final AHRS m_gyro;

  // Odometry class for tracking robot pose
  private DifferentialDriveOdometry m_odometry;

  private CSVWriter odometryWriter;
  private Path deployPath;

  public double leftOutputVoltage, rightOutputVoltage, leftSetpoint, rightSetpoint;
  public boolean odometryWriterActive, loggingEnabled = false;
  public double startingTime;
  
  private boolean driveInverted;

  /**
   * Creates a new DriveSubsystem.
   */
  public RobotDriveSubsystem() {
    this.leftDrive = new CANSparkMax(Constants.leftDrive, MotorType.kBrushless);
    this.leftDriveB = new CANSparkMax(Constants.leftDriveB, MotorType.kBrushless);

    this.leftDrive.setInverted(false);
    this.leftDriveB.setInverted(false);

    this.rightDrive = new CANSparkMax(Constants.rightDrive, MotorType.kBrushless);
    this.rightDriveB = new CANSparkMax(Constants.rightDriveB, MotorType.kBrushless);

    this.rightDrive.setInverted(true);
    this.rightDriveB.setInverted(true);
    
    setMotorIdleMode(IdleMode.kCoast);

    this.m_leftMotors = new SpeedControllerGroup(leftDrive, leftDriveB);
    this.m_rightMotors = new SpeedControllerGroup(rightDrive, rightDriveB);

    this.m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    this.m_leftEncoder = leftDrive.getEncoder();
    this.m_rightEncoder = rightDrive.getEncoder();
    this.m_gyro = new AHRS();

    resetEncoders();
    this.m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    this.deployPath = Filesystem.getDeployDirectory().toPath();
    File odometryLogging = new File(deployPath.resolve("csv/Odometry.csv").toString());
    try {
      this.odometryWriter = new CSVWriter(new FileWriter(odometryLogging));
      this.odometryWriterActive = true;
    } catch (IOException e) {
      System.err.println("Failed to open odometryWriter because:");
      System.err.println(e.toString());
      this.odometryWriterActive = false;
    }

    this.startingTime = System.currentTimeMillis();
  }

  public void enableLogging(boolean enabled) {
    this.loggingEnabled = enabled;
  }

  public void closeCSVs() {
    if(this.odometryWriterActive) {
      try {
        this.odometryWriter.close();
      } catch (IOException e) {
  
      }
    }
    this.odometryWriterActive = false;
  }

  @Override
  public void periodic() {
    double leftOutputRotations = Utilities.driveshaftIntputToOutput(this.m_leftEncoder.getPosition(), "low");
    double leftOutputMeters = Utilities.rotationsToMeter(leftOutputRotations);

    double rightOutputRotations = Utilities.driveshaftIntputToOutput(this.m_rightEncoder.getPosition(), "low");
    double rightOutputMeters = Utilities.rotationsToMeter(rightOutputRotations);

    this.m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftOutputMeters, rightOutputMeters);

    // SmartDashboard.putNumber("PoseX", getPose().getTranslation().getX());
    // SmartDashboard.putNumber("PoseY", getPose().getTranslation().getY());
    // SmartDashboard.putNumber("Left Encoder in Meters", leftOutputMeters);
    // SmartDashboard.putNumber("Right Encoder in Meters", rightOutputMeters);

    String recordsString = String.format("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", getPose().getTranslation().getX(), // PoseX
        getPose().getTranslation().getY(), // PoseY
        getHeading(), // PoseHeading
        getWheelSpeeds().leftMetersPerSecond, // Left Wheel Speed (m/s)
        getWheelSpeeds().rightMetersPerSecond, // Right Wheel Speed (m/s)
        this.leftOutputVoltage, // Left Voltage
        this.rightOutputVoltage, // Right Voltage
        this.leftSetpoint, // Left Setpoint (m/s)
        this.rightSetpoint, // Right Setpoint (m/s)
        System.currentTimeMillis()-this.startingTime);
    String[] records = recordsString.split(",");
    if (this.odometryWriterActive && this.loggingEnabled) {
      this.odometryWriter.writeNext(records);
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return this.m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    double leftVelocityRPM = this.m_leftEncoder.getVelocity();
    double rightVelocityRPM = this.m_rightEncoder.getVelocity();
    double leftVelocityMPS = Utilities.RPMtoMPS(leftVelocityRPM);
    double rightVelocityMPS = Utilities.RPMtoMPS(rightVelocityRPM);
    return new DifferentialDriveWheelSpeeds(leftVelocityMPS, rightVelocityMPS);
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    this.m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    this.m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    this.m_leftMotors.setVoltage(leftVolts);
    this.m_rightMotors.setVoltage(rightVolts);
    this.m_drive.feed();

    this.leftOutputVoltage = leftVolts;
    this.rightOutputVoltage = rightVolts;
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    this.m_leftEncoder.setPosition(0);
    this.m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (this.m_leftEncoder.getPosition() + this.m_rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public CANEncoder getLeftEncoder() {
    return this.m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public CANEncoder getRightEncoder() {
    return this.m_rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    this.m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    this.m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return this.m_gyro.getAngle() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return this.m_gyro.getRate() * (Constants.kGyroReversed ? -1.0 : 1.0);
  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    double deadzone = 0.1;
    leftSpeed = Utilities.adjustForDeadzone(leftSpeed, deadzone);
    rightSpeed = Utilities.adjustForDeadzone(rightSpeed, deadzone);
    
    this.setRelativeDrive(leftSpeed, rightSpeed);
  }
  private void setRelativeDrive(double leftSpeed, double rightSpeed) {
    if (this.driveInverted) {
      this.m_drive.tankDrive(-leftSpeed, rightSpeed);
    } else {
      this.m_drive.tankDrive(rightSpeed, -leftSpeed);
    }
  }

  public void setInvertedDrive(boolean invert) {
    this.driveInverted = invert;
    this.rightDrive.setInverted(!this.driveInverted);
    this.leftDrive.setInverted(this.driveInverted);
  }

  public void setMotorIdleMode(IdleMode mode) {
    this.leftDrive.setIdleMode(mode);
    this.leftDriveB.setIdleMode(mode);
    this.rightDrive.setIdleMode(mode);
    this.rightDriveB.setIdleMode(mode);
  }

  public void stop() {
    this.m_drive.tankDrive(0, 0);
  }
}