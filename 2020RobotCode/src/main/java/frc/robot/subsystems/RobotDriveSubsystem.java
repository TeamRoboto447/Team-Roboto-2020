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
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
//import edu.wpi.first.wpilibj.smartdashboard.SmaartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Utilities;
import frc.robot.utils.Ramp;

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

  private final Solenoid transmission;

  // Odometry class for tracking robot pose and speed
  private DifferentialDriveOdometry m_odometry;
  private DifferentialDriveKinematics kinematics;

  private CSVWriter odometryWriter;
  private Path deployPath;

  public double leftOutputVoltage, rightOutputVoltage, leftSetpoint, rightSetpoint;
  public boolean odometryWriterActive, loggingEnabled = false;
  public double startingTime;

  private boolean driveInverted;

  private Ramp rightRamp, leftRamp;
  /**
   * Creates a new DriveSubsystem.
   */
  public RobotDriveSubsystem() {
    this.leftDrive = new CANSparkMax(Constants.leftDrive, MotorType.kBrushless);
    this.leftDriveB = new CANSparkMax(Constants.leftDriveB, MotorType.kBrushless);

    this.rightDrive = new CANSparkMax(Constants.rightDrive, MotorType.kBrushless);
    this.rightDriveB = new CANSparkMax(Constants.rightDriveB, MotorType.kBrushless);

    this.rightRamp = new Ramp(Constants.driveTrainRampMaxPerSecond, Constants.driveTrainRampMaxTimeDelta, this.getCurrentGear() == "high");
    this.leftRamp = new Ramp(Constants.driveTrainRampMaxPerSecond, Constants.driveTrainRampMaxTimeDelta, this.getCurrentGear() == "high");

    setInvertedDrive(false);

    setMotorIdleMode(IdleMode.kCoast);

    this.transmission = new Solenoid(Constants.transmission);

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

    this.kinematics = new DifferentialDriveKinematics(Constants.kTrackWidthMeters);

    this.startingTime = System.currentTimeMillis();
  }

  public void enableLogging(boolean enabled) {
    this.loggingEnabled = enabled;
  }

  public void closeCSVs() {
    if (this.odometryWriterActive) {
      try {
        this.odometryWriter.close();
      } catch (IOException e) {

      }
    }
    this.odometryWriterActive = false;
  }

  @Override
  public void periodic() {

    double leftOutputRotations = Utilities.driveshaftInputToOutput(this.m_leftEncoder.getPosition(),
        this.getCurrentGear());
    double leftOutputMeters = Utilities.rotationsToMeter(leftOutputRotations);

    double rightOutputRotations = Utilities.driveshaftInputToOutput(this.m_rightEncoder.getPosition(),
        this.getCurrentGear());
    double rightOutputMeters = Utilities.rotationsToMeter(rightOutputRotations);

    this.m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftOutputMeters, rightOutputMeters);

    String recordsString = String.format("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", getPose().getTranslation().getX(), // PoseX
        getPose().getTranslation().getY(), // PoseY
        getHeading(), // PoseHeading
        getWheelSpeeds().leftMetersPerSecond, // Left Wheel Speed (m/s)
        getWheelSpeeds().rightMetersPerSecond, // Right Wheel Speed (m/s)
        this.leftOutputVoltage, // Left Voltage
        this.rightOutputVoltage, // Right Voltage
        this.leftSetpoint, // Left Setpoint (m/s)
        this.rightSetpoint, // Right Setpoint (m/s)
        System.currentTimeMillis() - this.startingTime);

    String[] records = recordsString.split(",");

    if (this.odometryWriterActive && this.loggingEnabled) {
      this.odometryWriter.writeNext(records);
    }

    switch(this.getCurrentGear()) {
      case "low":
        this.transmission.set(false);
        break;
      case "high":
        this.transmission.set(true);
        break;
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
    double leftVelocityMPS = Utilities.RPMtoMPS(leftVelocityRPM, this.getCurrentGear());
    double rightVelocityMPS = Utilities.RPMtoMPS(rightVelocityRPM, this.getCurrentGear());
    return new DifferentialDriveWheelSpeeds(leftVelocityMPS, rightVelocityMPS);
  }

  public ChassisSpeeds getChassisSpeed() {
    ChassisSpeeds speeds = kinematics.toChassisSpeeds(this.getWheelSpeeds());
    return speeds;
  }
  
  public double[] getFieldRelativeSpeed() {
    ChassisSpeeds chassisSpeeds = this.getChassisSpeed();
    double angle = this.getHeading();
    double Vx = Math.sin(Math.toRadians(angle)) * chassisSpeeds.vxMetersPerSecond;
    double Vy = Math.cos(Math.toRadians(angle)) * chassisSpeeds.vxMetersPerSecond;
    double[] speeds = new double[]{Vx, Vy};
    return speeds;
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
    double leftSpeed = fwd - rot;
    double rightSpeed = fwd + rot;
    this.setRelativeDrive(leftSpeed, rightSpeed);
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

    this.leftRamp.setEnable(this.getCurrentGear() == "high");
    this.rightRamp.setEnable(this.getCurrentGear() == "high");

    leftSpeed = this.leftRamp.run(leftSpeed);
    rightSpeed = this.rightRamp.run(rightSpeed);

    this.setRelativeDrive(leftSpeed, rightSpeed);
  }

  private void setRelativeDrive(double leftSpeed, double rightSpeed) {
    if (this.driveInverted) {
      this.m_drive.tankDrive(leftSpeed, -rightSpeed);
    } else {
      this.m_drive.tankDrive(rightSpeed, -leftSpeed);
    }
  }

  public void setInvertedDrive(boolean invert) {
    this.driveInverted = invert;
    this.rightDrive.setInverted(!invert);
    this.rightDriveB.setInverted(!invert);
    this.leftDrive.setInverted(invert);
    this.leftDriveB.setInverted(invert);
  }

  public boolean getInvertedDrive() {
    return this.driveInverted;
  }

  String currentGear = "low";

  public void setCurrentGear(String gear) {
    this.currentGear = gear.toLowerCase();
  }

  public String getCurrentGear() {
    String output;

    switch(this.currentGear) {
      case "low":
        output = "low";
        break;
      case "high":
        output = "high";
        break;
      default:
        output = "low";
    }

    return output;
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