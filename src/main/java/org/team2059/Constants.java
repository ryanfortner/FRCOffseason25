// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosly.
 */
public final class Constants {

  public static class OculusConstants {
    public static final Time connectionTimeout = Milliseconds.of(350);

    /**
     * Standard deviations representing how much we "trust" the position from the Oculus. By default,
     * the Quest 3 provides sub-centimeter accuracy. Values represent: [0]: X position trust (50mm)
     * [1]: Y position trust (50mm) [2]: Rotation trust (~2.87 degrees)
     */
    public static final Matrix<N3, N1> stdDevs = VecBuilder.fill(
      0.02, // Trust down to 50mm
      0.02, // Trust down to 50mm
      0.0872665
      // 5deg
    );

    /**
     * Transform from the robot center to the headset. Coordinate system: - X: Positive is forwards -
     * Y: Positive is left - Rotation: Positive is counter-clockwise
     */
    public static final Transform2d ROBOT_TO_OCULUS =
      new Transform2d(0.25, 0.0, Rotation2d.fromDegrees(0));

    public class Miso {
      /** Status indicating system is ready for commands */
      public static final int STATUS_READY = 0;

      /** Status indicating heading reset completion */
      public static final int STATUS_HEADING_RESET_COMPLETE = 99;

      /** Status indicating pose reset completion */
      public static final int STATUS_POSE_RESET_COMPLETE = 98;

      /** Status indicating ping response receipt */
      public static final int STATUS_PING_RESPONSE = 97;
    }

    public class Mosi {
      /** Clear status */
      public static final int COMMAND_CLEAR = 0;

      /** Command to reset the heading */
      public static final int COMMAND_RESET_HEADING = 1;

      /** Command to reset the pose */
      public static final int COMMAND_RESET_POSE = 2;

      /** Command to ping the system */
      public static final int COMMAND_PING = 3;
    }
  }

  public static class OperatorConstants {

    // Sets whether or not tunable numbers can be changed. If false, only defaults will be used.
    public static final boolean tuningMode = true;

    // Whether to use XboxController or Joystick
    public static final boolean useXboxForDriving = false;

    /* ===== */
    /* PORTS */
    /* ===== */

    public static final int xboxDriverPort = 3;
    public static final int logitechPort = 0;
    public static final int buttonBoxPort = 1;
    public static final int xboxControllerPort = 2;


    /* ==== */
    /* AXES */
    /* ==== */

    public static final int JoystickTranslationAxis = 1;
    public static final int JoystickStrafeAxis = 0;
    public static final int JoystickRotationAxis = 2;
    public static final int JoystickSliderAxis = 3;

    public static final int XboxDriverLeftYAxis = 1;     // Forward/backward 
    public static final int XboxDriverLeftXAxis = 0;     // Strafe
    public static final int XboxDriverRightXAxis = 4;    // Rotation

    /* ======= */
    /* BUTTONS */
    /* ======= */

    // We'll add these later once the button box is finalized
    public static final int JoystickResetHeading = 5;
    public static final int JoystickRobotRelative = 6;
    public static final int JoystickInvertedDrive = 4;
    public static final int JoystickStrafeOnly = 3;
  }

  public static class DrivetrainConstants {

    public static final Distance wheelBase = Inches.of(24.75); // Distance from center of wheels on side
    public static final Distance trackWidth = Inches.of(20.75); // Distance between front wheels (like train track)

    // Diameter of swerve module wheel
    public static final Distance wheelDiameter = Inches.of(4.0);

    // Kinematics gets each module relative to center. X is forward/backward and Y is left/right.
    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0), // front right (+,+)
      new Translation2d(wheelBase.in(Meters) / 2.0, -trackWidth.in(Meters) / 2.0), // back right (+,-)
      new Translation2d(-wheelBase.in(Meters) / 2.0, trackWidth.in(Meters) / 2.0), // front left (-,+)
      new Translation2d(-wheelBase.in(Meters) / 2.0, -trackWidth.in(Meters) / 2.0) // back left (-,-)
    );

    /* =========== */
    /* GEAR RATIOS */
    /* =========== */

    public static final double driveGearRatio = (5.9); // 5.9:1
    public static final double rotationGearRatio = (18.75); // 18.75

    /* ================== */
    /* CONVERSION FACTORS */
    /* ================== */

    // Given Motor Rotations, convert to Meters traveled
    // (pi * d) / (Gear Ratio)
    // where d is wheel diameter, in meters
    public static final double driveEncoderPositionConversionFactor = (Math.PI * Units.inchesToMeters(4)) / (6.75);
    // Given Motor RPM, convert to Meters/second
    public static final double driveEncoderVelocityConversionFactor = driveEncoderPositionConversionFactor / 60.0;
    // Given Motor Rotations, convert to Radians
    // (2 * pi) / (Gear Ratio)
    public static final double rotationEncoderPositionConversionFactor = 14.0 * Math.PI / 150.0;
    // Given Motor RPM, convert to Radians/second
    public static final double rotationEncoderVelocityConversionFactor = rotationEncoderPositionConversionFactor / 60.0;

    /* ============== */
    /* SWERVE MODULES */
    /* ============== */

    /*
     * CAN IDs: found and set via REV hardware client
     * CANcoder Offsets: found in Phoenix Tuner X as "Absolute position"
     *  after manually straightening wheel (converted to radians here by multiplying by 2pi)
     */

    // front left
    public static final int frontLeftDriveMotorId = 4;
    public static final int frontLeftRotationMotorId = 3;
    public static final int frontLeftCanCoderId = 10;
    public static final double frontLeftOffsetRad = 0.486572 * 2 * Math.PI;
    // front right
    public static final int frontRightDriveMotorId = 5;
    public static final int frontRightRotationMotorId = 6;
    public static final int frontRightCanCoderId = 20;
    public static final double frontRightOffsetRad = 0.740479 * 2 * Math.PI;
    // back left
    public static final int backLeftDriveMotorId = 8;
    public static final int backLeftRotationMotorId = 7;
    public static final int backLeftCanCoderId = 30;
    public static final double backLeftOffsetRad = 0.043213 * 2 * Math.PI;
    // back right
    public static final int backRightDriveMotorId = 1;
    public static final int backRightRotationMotorId = 2;
    public static final int backRightCanCoderId = 40;
    public static final double backRightOffsetRad = 0.369141 * 2 * Math.PI;

    /* ======== */
    /* MAXIMUMS */
    /* ======== */

    // Global maximums
    public static final double maxVelocity = 5; // meters/sec
    public static final double maxAcceleration = 10; // meters/sec^2
    public static final double maxAngularVelocity = 2 * Math.PI; // rad/sec
    public static final double maxAngularAcceleration = 4 * Math.PI; // rad/sec^2
    // Teleop max speeds
    public static final double kTeleDriveMaxSpeed = 4;
    public static final double kTeleDriveMaxAngularSpeed = Math.PI;

    /* =============================== */
    /* SWERVE MODULE CONTROL CONSTANTS */
    /* =============================== */
    
    public static final double kPRotation = 0.25;
    // kS: voltage needed to overcome static friction
    // kV: voltage needed to run at constant velocity
    // kA: voltage needed to accelerate
    public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.17821, 1.9047, 0.14686);
    public static final SimpleMotorFeedforward turnFF = new SimpleMotorFeedforward(0, 0, 0);
  }

  public static class AutoConstants {

    // Feedback constants for x & y translation in auto.
    public static final double kAutoTranslationP = 5.0;
    public static final double kAutoTranslationD = 0;

    // Feedback constants for theta (rotation) in auto.
    public static final double kAutoRotationP = 5.0;
    public static final double kAutoRotationD = 0.0;

    /* FOR ROBOTCONFIG AUTO STUFF... */
    /* Not used right now. */
    // public static final double kMass = 30;
    // public static final double kMomentOfIntertia = 3;
    
    // // CoF taken from https://www.chiefdelphi.com/t/coefficient-of-friction/467778
    // public static final double kWheelCoF = 1.542; // Coefficient of friction of wheels
    // public static final int driveCurrentLimit = 40;
    // public static final int turnCurrentLimit = 20;
  }
}
