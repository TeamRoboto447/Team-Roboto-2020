/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
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
        public static int loggingLevel = 4; // Logging level (-1 = no logging, 0 = errors only, 1 = +warnings, 2 =
        // +debug, 3
        // = +info)

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
                        climberSparkMax = 10, // Climber Spark Max, CAN ID 10.
                        intakeSpark = 0, // Intake Spark, PWM 0.
                        shooterFeedSpark = 1; // Shooter Feed Spark, PWM 1.

        // Targetting Light Relay
        public static final int lightRelay = 0;

        public static final int // Set safe current max for NEO motors (amps)
        miniNeoSafeAmps = 15, neoSafeAmps = 50;

        public static final double
        // Gear ratios for calculating encoder ticks per rotation
        lowGearRatio = 25.9, highGearRatio = 8.63, thirdStageRatio = 1.41,
                        // Wheel diameter for calculating inches per rotation
                        wheelDiameter = 6.0, wheelDiameterMeters = 0.1524;
        public static final double turretToMoterRatio = 100.0;

        // PCM Channels on board 1 (pneumatics)
        public static final int transmissionLow = 0, // Drive transmission low channel, PCM channel 0.
                        transmissionHigh = 1; // Drive transmission high channel, PCM channel 1.

        public static final double drivekP = 0.1, drivekI = 0, // 1e-4
                        drivekD = 1, drivekIz = 0, drivekFF = 0, drivePIDMin = -0.5, drivePIDMax = 0.5;

        // Shooter PID info
        public static final double 
                        shooterkP = 0.000270,
                        shooterkI = 0.001163,
                        shooterkD = 0.000016,
                        shooterkFFm = 0.000186,
                        shooterkFFb = -0.018051, 
                        shooterIZone = 150,
                        shooterSZone = 100,
                        speedkM = 0.0078859,
                        speedkB = 0.6975444,
                        shooterPidIntegralResetTime = 2,
                        distanceFromInnerToOuterPort = 29.5/12,
                        maxInnerPortAjustmentAngle = Math.PI / 4,
                        shooterMarginOfError = 60;

        public static final double
        distanceLineEqM = 3.322622173196587,
        distanceLineEqB = -1.5053081308055845;


        // Turning PID info
        public static final double turretkP = 0.012000, turretkI = 0.072816, turretkD = 0.0007, turretFFm = 0, turretFFb = 0,
                        turretIZone = 10, turretSpinLimit = 190;

        public static final double encoderRes = 4096;

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
