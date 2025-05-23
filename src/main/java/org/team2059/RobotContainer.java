// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.team2059;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;


import org.team2059.Constants.OperatorConstants;

import org.team2059.commands.drive.TeleopDriveCmd;
import org.team2059.commands.drive.TeleopDriveCmdXbox;
import org.team2059.subsystems.drive.Drivetrain;
import org.team2059.subsystems.drive.GyroIONavX;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  SendableChooser<Command> autoChooser;

  public static Drivetrain drivetrain;

  public static CommandXboxController xboxDriver;
  public static Joystick logitech;

  public static Supplier<Double> strafe; 
  public static Supplier<Double> translation; 
  public static Supplier<Double> rotation; 

  public static boolean isRed = false; 

  public static boolean isSlowMode;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    isSlowMode = true;

    /* ========== */
    /* SUBSYSTEMS */
    /* ========== */

    drivetrain = new Drivetrain(new GyroIONavX());

    /* =========== */
    /* CONTROLLERS */
    /* =========== */
    
    xboxDriver = new CommandXboxController(OperatorConstants.xboxDriverPort);
    logitech = new Joystick(OperatorConstants.logitechPort);

    // Driver Controls
    if (OperatorConstants.useXboxForDriving) {
      translation = xboxDriver::getLeftX;
      strafe = xboxDriver::getLeftY;
      rotation = xboxDriver::getRightX;
    }

    /* ================ */
    /* DEFAULT COMMANDS */
    /* ================ */

    // Default commands run when the subsystem in question has no scheduled commands requiring it.
    if (OperatorConstants.useXboxForDriving) {
      drivetrain.setDefaultCommand(
      new TeleopDriveCmdXbox(
        drivetrain, 
        () -> -strafe.get(), // forwardX
        () -> -translation.get(), // forwardY
        () -> -rotation.get(), // rotation
        () -> isSlowMode
      )); 
    } else {
      drivetrain.setDefaultCommand(
        new TeleopDriveCmd(
          drivetrain,
          () -> -logitech.getRawAxis(OperatorConstants.JoystickTranslationAxis), // forwardX
          () -> -logitech.getRawAxis(OperatorConstants.JoystickStrafeAxis), // forwardY
          () -> logitech.getRawAxis(OperatorConstants.JoystickRotationAxis), // rotation
          () -> logitech.getRawAxis(OperatorConstants.JoystickSliderAxis), // slider
          () -> logitech.getRawButton(OperatorConstants.JoystickStrafeOnly), // Strafe Only Button
          () -> logitech.getRawButton(OperatorConstants.JoystickInvertedDrive) // Inverted buytton
        )
      );
    }

    /* ========== */
    /* AUTONOMOUS */
    /* ========== */

    // Build auto chooser - you can also set a default.
    autoChooser = AutoBuilder.buildAutoChooser();

    /* ======= */
    /* LOGGING */
    /* ======= */

    // Build info
    SmartDashboard.putString("ProjectName", "Offseason25");
    SmartDashboard.putString("BuildDate", BuildConstants.BUILD_DATE);
    SmartDashboard.putString("GitSHA", BuildConstants.GIT_SHA);
    SmartDashboard.putString("GitDate", BuildConstants.GIT_DATE);
    SmartDashboard.putString("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        SmartDashboard.putString("GitDirty", "All changes committed");
        break;
      case 1:
        SmartDashboard.putString("GitDirty", "Uncommitted changes");
        break;
      default:
        SmartDashboard.putString("GitDirty", "Unknown");
        break;
    }

    // Allow viewing of command scheduler queue in dashboards 
    SmartDashboard.putData(CommandScheduler.getInstance());

    // Publish subsystem status to dashboard
    SmartDashboard.putData(drivetrain);

    // Publish auto chooser
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();

  }

  public void slowMode() {
    isSlowMode = !isSlowMode;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    if (OperatorConstants.useXboxForDriving) {
      xboxDriver.start().onTrue(new InstantCommand(() -> drivetrain.zeroHeading()));
      xboxDriver.back().onTrue(new InstantCommand(() -> drivetrain.setFieldRelativity()));
      xboxDriver.rightBumper().whileTrue(new InstantCommand(this::slowMode));
    } else {
      /* RESET NAVX HEADING */
      new JoystickButton(logitech, OperatorConstants.JoystickResetHeading)
        .whileTrue(new InstantCommand(() -> drivetrain.zeroHeading()));

      /* SWITCH FIELD/ROBOT RELATIVITY IN TELEOP */
      new JoystickButton(logitech, OperatorConstants.JoystickRobotRelative)
        .whileTrue(new InstantCommand(() -> drivetrain.zeroPose()));
    }

    /* ========== */
    /* Drivetrain */
    /* ========== */

    // Gyro reset & field relative buttons are above (switched for Xbox and Logitech controllers)

    // Drivetrain translation sysID routine (just drive motors) (wheels must be locked straight for this)
    // new JoystickButton(buttonBox, 1)
    //   .whileTrue(drivetrain.routine.quasistaticForward());
    // new JoystickButton(buttonBox, 2)
    //   .whileTrue(drivetrain.routine.quasistaticReverse());
    // new JoystickButton(buttonBox, 3)
    //   .whileTrue(drivetrain.routine.dynamicForward());
    // new JoystickButton(buttonBox, 4)
    //   .whileTrue(drivetrain.routine.dynamicReverse());

   }

   public Drivetrain getDrivetrainSubsystem() {
    return drivetrain;
   }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}