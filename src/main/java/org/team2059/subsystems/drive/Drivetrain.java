// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059.subsystems.drive;

import edu.wpi.first.math.kinematics.*;
import org.littletonrobotics.junction.Logger;
import org.team2059.Constants;
import org.team2059.Constants.AutoConstants;
import org.team2059.Constants.DrivetrainConstants;
import org.team2059.routines.DrivetrainRoutine;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team2059.subsystems.oculus.QuestNav;

public class Drivetrain extends SubsystemBase {

  public static boolean fieldRelativeStatus = true;

  public final SwerveModule frontLeft;
  public final SwerveModule frontRight;
  public final SwerveModule backLeft;
  public final SwerveModule backRight;

  private final GyroIO gyro;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  // private final SwerveDriveOdometry odometry;

  public final DrivetrainRoutine routine;

  private final Field2d field;

  private final QuestNav questNav = new QuestNav();

  private final SwerveDrivePoseEstimator poseEstimator; 

  public Drivetrain(GyroIO gyro) {

    /*
     * Construct four SwerveModules
     * 
     * Arguments: ID, then SwerveModuleIO:
     * - Drive motor can ID
     * - Rotation motor can ID
     * - Cancoder can ID
     * - Cancoder offset in radians
     * - Boolean drive inverter
     * - Boolean rotation inverter
     * - kS, kV, kA constants for drive feedforward (velocity control)
     * - kP constant for drive (velocity control)
     * 
     * 1 frontLeft
     * 2 frontRight
     * 3 backLeft
     * 4 backRight
     */

    frontLeft = new SwerveModule(
        1,
        new SwerveModuleIOReal(
            DrivetrainConstants.frontLeftDriveMotorId,
            DrivetrainConstants.frontLeftRotationMotorId,
            DrivetrainConstants.frontLeftCanCoderId,
            DrivetrainConstants.frontLeftOffsetRad,
            true,
            true,
            0.18707,
            1.972,
            0.2846,
            0.0));
    frontRight = new SwerveModule(
        2,
        new SwerveModuleIOReal(
            DrivetrainConstants.frontRightDriveMotorId,
            DrivetrainConstants.frontRightRotationMotorId,
            DrivetrainConstants.frontRightCanCoderId,
            DrivetrainConstants.frontRightOffsetRad,
            true,
            true,
            0.17367,
            2.0218,
            0.30097,
            0.0));
    backLeft = new SwerveModule(
        3,
        new SwerveModuleIOReal(
            DrivetrainConstants.backLeftDriveMotorId,
            DrivetrainConstants.backLeftRotationMotorId,
            DrivetrainConstants.backLeftCanCoderId,
            DrivetrainConstants.backLeftOffsetRad,
            true,
            true,
            0.1846,
            1.9744,
            0.28488,
            0.0));
    backRight = new SwerveModule(
        4,
        new SwerveModuleIOReal(
            DrivetrainConstants.backRightDriveMotorId,
            DrivetrainConstants.backRightRotationMotorId,
            DrivetrainConstants.backRightCanCoderId,
            DrivetrainConstants.backRightOffsetRad,
            true,
            true,
            0.16226,
            2.0166,
            0.27832,
            0.0));

    // odometry = new SwerveDriveOdometry(
    //   DrivetrainConstants.kinematics,
    //   getHeading(),
    //   getModulePositions()
    // );

    poseEstimator = 
      new SwerveDrivePoseEstimator(DrivetrainConstants.kinematics, getHeading(), getModulePositions(), new Pose2d()); 

    // Gyro keeps track of field-relative rotation
    this.gyro = gyro;
    new Thread(() -> { // gyro may need an extra second to start...
      try {
        Thread.sleep(1000);
        gyro.reset();
      } catch (Exception e) {
        e.printStackTrace();
      }
    }).start();

    // initialize CANcoder offsets
    frontLeft.io.initRotationOffset();
    frontRight.io.initRotationOffset();
    backLeft.io.initRotationOffset();
    backRight.io.initRotationOffset();

    // reset encoders upon each start
    frontLeft.io.resetEncoders();
    frontRight.io.resetEncoders();
    backLeft.io.resetEncoders();
    backRight.io.resetEncoders();

    // SysID routine
    routine = new DrivetrainRoutine(this);

    // Configure auto builder last
    configureAutoBuilder();

    field = new Field2d();

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> { // target pose
      field.getObject("target pose").setPose(pose);
    });
    PathPlannerLogging.setLogActivePathCallback((poses) -> { // active path (list of poses)
      field.getObject("trajectory").setPoses(poses);
    });

    SmartDashboard.putData(field);
  }

  /**
   * @return Current robot pose in meters
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Reset odometry to a certain pose,
   * uses current module positions and heading
   * 
   * @param pose specified Pose2d
   */
  public void resetOdometry(Pose2d pose) {
    // odometry.resetPosition(getHeading(), getModulePositions(), pose);
    poseEstimator.resetPosition(getHeading(), getModulePositions(), pose);
  }

  /**
   * @return ChassisSpeeds of current robot-relative speeds
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return DrivetrainConstants.kinematics.toChassisSpeeds(getStates());
  }

  /**
   * Set reported heading to zero
   */
  public void zeroHeading() {
    questNav.setHeading(0);
  }

  /**
   * Set reported heading to an arbitrary value - be careful!
   */
  public void setHeading(double degrees) {
    questNav.setHeading(degrees);
  }

  /**
   * Syncs Quest with onboard IMU (this should be called every once in a while, not in every periodic loop)
   */
  public void syncQuestHeading() {
    questNav.setHeading(-gyroInputs.yaw);
  }

  /**
   * @return Rotation2d of current reported heading
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyroInputs.yaw);
  }

  public void zeroPose() {
    questNav.setPose(Pose2d.kZero);
    poseEstimator.resetPose(new Pose2d());
  }

  public void setPose(Pose2d targetPose) {
    questNav.setPose(targetPose);
  }

  /**
   * @return current swerve module positions in SwerveModulePosition[] array
   */
  public SwerveModulePosition[] getModulePositions() {

    return new SwerveModulePosition[]{
        new SwerveModulePosition(frontLeft.inputs.drivePosition,
            new Rotation2d(frontLeft.inputs.rotationAbsolutePositionRadians)),
        new SwerveModulePosition(frontRight.inputs.drivePosition,
            new Rotation2d(frontRight.inputs.rotationAbsolutePositionRadians)),
        new SwerveModulePosition(backLeft.inputs.drivePosition,
            new Rotation2d(backLeft.inputs.rotationAbsolutePositionRadians)),
        new SwerveModulePosition(backRight.inputs.drivePosition,
            new Rotation2d(backRight.inputs.rotationAbsolutePositionRadians))
    };
  }

  /**
   * Method to drive robot-relative
   * 
   * @param chassisSpeeds
   */
  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    SwerveModuleState[] newStates = Constants.DrivetrainConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.DrivetrainConstants.maxVelocity);
    setModuleStates(newStates);
  }

  /**
   * Method to drive field-relative
   * 
   * @param chassisSpeeds
   */
  public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
    chassisSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(discreteSpeeds, getHeading());
    SwerveModuleState[] newStates = Constants.DrivetrainConstants.kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(newStates, Constants.DrivetrainConstants.maxVelocity);
    setModuleStates(newStates);
  }

  /**
   * @return current swerve module states of all modules
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = frontLeft.io.getState();
    states[1] = frontRight.io.getState();
    states[2] = backLeft.io.getState();
    states[3] = backRight.io.getState();

    return states;
  }

  /**
   * Method to set module states
   * 
   * @param desiredStates SwerveModuleState[] desired states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // makes it never go above specified max velocity
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConstants.maxVelocity);

    Logger.recordOutput("Desired States", desiredStates);

    // Sets the speed and rotation of each module
    frontLeft.io.setState(desiredStates[0], false);
    frontRight.io.setState(desiredStates[1], false);
    backLeft.io.setState(desiredStates[2], false);
    backRight.io.setState(desiredStates[3], false);
  }

  /**
   * Method to drive the robot either field or robot relative
   * 
   * @param forward
   * @param strafe
   * @param rotation
   * @param isFieldRelative
   */
  public void drive(double forward, double strafe, double rotation, boolean isFieldRelative) {

    /**
     * ChassisSpeeds object to represent the overall state of the robot
     * ChassisSpeeds takes a forward and sideways linear value and a rotational
     * value
     * 
     * speeds is set to field relative or default (robot relative) based on
     * parameter
     */

    ChassisSpeeds speeds = isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(forward, strafe, rotation, getHeading())
        : new ChassisSpeeds(forward, strafe, rotation);

    speeds = ChassisSpeeds.discretize(speeds, 0.02);

    // use kinematics (wheel placements) to convert overall robot state to array of
    // individual module states
    SwerveModuleState[] states = DrivetrainConstants.kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.maxVelocity);

    setModuleStates(states);

  }

  /**
   * Switch fieldRelativeStatus boolean to the opposite value
   */
  public void setFieldRelativity() {
    if (fieldRelativeStatus) {
      fieldRelativeStatus = false;
    } else {
      fieldRelativeStatus = true;
    }
  }

  /**
   * Method to configure AutoBuilder (make sure to do this last)
   */
  public void configureAutoBuilder() {

    System.out.println("Configuring Auto Builder...");

    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
          this::getPose, // Robot pose supplier
          this::resetOdometry, // Method to reset odometry
          this::getRobotRelativeSpeeds, // ChassisSpeeds supplier, MUST be robot relative
          (speeds) -> driveRobotRelative(speeds), // Method that will drive the robot given robot-relative chassisspeeds
          new PPHolonomicDriveController(
              new PIDConstants(AutoConstants.kAutoTranslationP, 0.0, AutoConstants.kAutoTranslationD),
              new PIDConstants(AutoConstants.kAutoRotationP, 0.0, AutoConstants.kAutoRotationD)),
          config,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field
            // The origin will remain on the blue side
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  // Stop all motors in every swerve module
  public void stopAllMotors() {
    frontLeft.io.stop();
    frontRight.io.stop();
    backLeft.io.stop();
    backRight.io.stop();
  }

  // For drivetrain translation routine. Must lock wheels, so instead we use PID
  public void setModulesToZeroRadPID() {
    frontLeft.io.setRotationMotorAnglePID(0);
    frontRight.io.setRotationMotorAnglePID(0);
    backLeft.io.setRotationMotorAnglePID(0);
    backRight.io.setRotationMotorAnglePID(0);
  }

  public void set180GyroRotation(boolean enabled) {
    gyro.set180Rotation(enabled);
  }

  @Override
  public void periodic() {

    // Update gyro inputs & logging
    gyro.updateInputs(gyroInputs);
    Logger.processInputs("Gyro", gyroInputs);

    // For safety...
    if (DriverStation.isDisabled()) {
      stopAllMotors();
    }

    
    poseEstimator.addVisionMeasurement(getPose(), questNav.getTimestamp(), Constants.OculusConstants.stdDevs);

    poseEstimator.updateWithTime(questNav.getTimestamp(), getHeading(), getModulePositions());
    


    // Logging
    Logger.recordOutput("Pose", getPose());
    field.setRobotPose(getPose());
    Logger.recordOutput("Field-Relative?", fieldRelativeStatus);
    Logger.recordOutput("Real States", getStates());

    questNav.processHeartbeat();
    questNav.cleanupResponses();

    Logger.recordOutput("QuestPose", questNav.getPose());
    Logger.recordOutput("QuestYaw", questNav.getYaw());
  }
}
