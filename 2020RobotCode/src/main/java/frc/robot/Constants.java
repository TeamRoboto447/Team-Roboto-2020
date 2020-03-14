/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                   */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static int loggingLevel = 4; // Logging level (-1 = no logging, 0 = errors only, 1 = +warnings, 2 = +debug, 3
                                        // = +info)
    public static Boolean enableCSVLogging = true;
    public static int noLogging = -1, errors = 0, warnings = 1, debug = 2, info = 3;

    public static final boolean bypassShooterPID = false;

    public static final int ballIntake = 0; // Ball intake spark, PWM channel 0.

    // Motors
    public static final int leftDrive = 3, // Left Drive Spark Max, CAN ID 1
            leftDriveB = 4, // Left Drive Spark Max, CAN ID 2.
            rightDrive = 1, // Right Drive Spark Max, CAN ID 3.
            rightDriveB = 2, // Right Drive Spark Max, CAN ID 4.
            turretSparkMax = 5, // Turret Rotation Spark Max, CAN ID 5.
            shooterSparkMaxLeft = 6, // Shooter Left Spark Max, CAN ID 6.
            shooterSparkMaxRight = 7, // Shooter Right Spark Max, CAN ID 7.
            indexingSparkMax = 8, // Indexer Spark Max, CAN ID 8.
            wheelOfFortuneSparkMax = 9, // WOF Spark Max, CAN ID 9.
            intakeSpark = 0, // Intake Spark, PWM 0.
            shooterFeedSpark = 1, // Shooter Feed Spark, PWM 1.
            climberSpark = 2; // Climber Spark, PWM 2.

    // PCM Channels on board 1 (pneumatics)
    public static final int transmission = 0, // Drive transmission, PCM channel 0.
            intakeExtension = 1, // Intake extension, PCM channel 1.
            climberLock = 2, // Climber lock, PCM channel 2.
            climberLift = 3, // Climber lift cylinder, PCM channel 3.
            climberRelease = 4; // Climber release, PCM channel 4.

    public static final int indexerFirstPos = 0, // Indexing sensor sensor first position, DIO channel 0.
            indexerSecondPos = 1, // Indexing sensor second position, DIO channel 1.
            fullIndexerSensor = 2; // Full Indexer Sensor , DIO channel 2.

    // Targetting Light Relay

    // Targetting Light Relay
    public static final int lightRelay = 0;

    public static final double intakeSpeed = 0.8, indexingSpeed = 0.8;

    public static final int // Set safe current max for NEO motors (amps)
    miniNeoSafeAmps = 15, neoSafeAmps = 40;

    public static final double
    // Gear ratios for calculating encoder ticks per rotation
    lowGearRatio = 25.9, highGearRatio = 6.86, thirdStageRatio = 1.14,
            // Wheel diameter for calculating inches per rotation
            wheelDiameter = 6.0, wheelDiameterMeters = 0.1524, shooterWheelDiameter = 4;
    public static final double turretToMoterRatio = 100.0;

    public static final double 
        drivekP = 0.1,
        drivekI = 0, // 1e-4
        drivekD = 0.5,
        drivekIz = 0,
        drivekFF = 0,
        steerkP = 0.0005,
        steerkI = 0,
        steerkD = 0;

    // Shooter PID info
    public static final double shooterkP = 2.7e-04,
        shooterkI = 1.077605e-03,
        shooterkD = 1.69125e-05,
        shooterkFFm = 1.734411e-04,
        shooterkFFb = -1.372668e-02, 
        shooterIZone = 100,
        shooterSZone = 100,
        speedkM = 0.014566739586865,
        speedkB = 0.427238873791768,
        shooterPidIntegralResetTime = 2,
        distanceFromInnerToOuterPort = 29.5 / 12,
        maxInnerPortAjustmentAngle = Math.PI / 4,
        shooterMarginOfError = 60;

    public static final double distanceLineEqM = 4.076545627333885, distanceLineEqB = -4.046285991880239;

    // Turning PID info
    public static final double
        turretkP = 0.03,
        turretkI = 0.04,
        turretkD = 0.0007,
        turretFFm = 0,
        turretFFb = 0,
        turretIZone = 10,
        turretSpinLimit = 125,
        turretMarginOfError = 2,
        staticAimOffset = 4;

    public static final double encoderRes = 1;

    public static final double 
        driveTrainRampTimeToMax = 0.75,
        driveTrainRampMaxTimeDelta = 0.4; 

    // Temporary for testing:
    public static final boolean kGyroReversed = true;

    public static final double ksVolts = 0.122;
    public static final double kvVoltSecondsPerMeter = 6.68;
    public static final double kaVoltSecondsSquaredPerMeter = 0.604;
    public static final double kPDriveVel = 0.005;
    public static final double kIDriveVel = 0.0;
    public static final double kDDriveVel = 0.01;

    public static final double kTrackWidthMeters = 0.693702941073319;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackWidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 2.1336;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
