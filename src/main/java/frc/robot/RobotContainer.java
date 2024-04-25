// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.intakeConstants;
import frc.robot.Constants.passThroughConstants;
import frc.robot.Constants.shooterConstants;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.pivot_down;
import frc.robot.subsystems.pivot_up;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.passthrough;
import frc.robot.subsystems.shooter;

// path planner
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  // /* Setting up bindings for necessary control of the swerve drive platform */
  // private final CommandJoystick joystick1 = new CommandJoystick(1); // My joystick
  private final CommandXboxController joystick1 = new CommandXboxController(0);
  private final CommandXboxController joystick2 = new CommandXboxController(1);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final pivot_down m_pivot_down = new pivot_down();
  private final pivot_up m_pivot_up = new pivot_up();
  private final intake m_intake = new intake(new TalonFX(intakeConstants.intake_ID));
  private final passthrough m_passthrough = new passthrough(new TalonFX(passThroughConstants.pass_ID));
  private final shooter m_shooter = new shooter(new TalonFX(shooterConstants.shooter_ID_lo), new TalonFX(shooterConstants.shooter_ID_hi));

  // autonomous chooser/selector
  private final SendableChooser<Command> autoChooser;
  
  // some motoors
  // private TalonFX m_climb_motor = new TalonFX(20);
  private TalonFX m_pivot_right = new TalonFX(17);
  private TalonFX m_pivot_left = new TalonFX(18);

  // bindings and control
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick1.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick1.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick1.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    /*
     * CONTROLS: PIVOT SYSTEM
     */
    joystick2.rightBumper().onTrue(
      Commands.runOnce(() -> {
        m_pivot_up.setGoal(1.2);
        m_pivot_up.enable();
      },
      m_pivot_up));

    joystick2.rightBumper().onFalse(
      Commands.runOnce(() -> {
        m_pivot_up.disable();
        m_pivot_left.setVoltage(0);
        m_pivot_right.setVoltage(0);
      },
      m_pivot_up));
    
    joystick2.leftBumper().onTrue(
      Commands.runOnce(() -> {
        m_pivot_down.setGoal(-0.2);
        m_pivot_down.enable();
      },
      m_pivot_down));
      //.whileFalse(Commands.runOnce(() -> {m_pivot.disable();}, m_pivot));

    joystick2.leftBumper().onFalse(
      Commands.runOnce(() -> {
        m_pivot_down.disable();
        m_pivot_left.setVoltage(0);
        m_pivot_right.setVoltage(0);
      },
      m_pivot_down));

    joystick2.a().onTrue(
      Commands.runOnce(() -> {
        m_passthrough.setspeed(0.25);
      }, m_passthrough)
    );
    
    joystick2.a().onFalse(
      Commands.runOnce(() -> {
        m_passthrough.setspeed(0.0);
      }, m_passthrough)
    );

    joystick2.b().onTrue(
      Commands.runOnce(() -> {
        m_passthrough.setspeed(-0.4);
      }, m_passthrough)
    );
    
    joystick2.b().onFalse(
      Commands.runOnce(() -> {
        m_passthrough.setspeed(0.0);
      }, m_passthrough)
    );

    joystick2.rightTrigger().onTrue(
      Commands.runOnce(() -> {
        m_shooter.shootit();
      }, m_shooter)
    );

    joystick2.rightTrigger().onFalse(
      Commands.runOnce(() -> {
        m_shooter.stop();
      }, m_shooter)
    );

    joystick2.leftTrigger().onTrue(
      Commands.runOnce(() -> {
        m_shooter.eatit();
      }, m_shooter)
    );

    joystick2.leftTrigger().onFalse(
      Commands.runOnce(() -> {
        m_shooter.stop();
      }, m_shooter)
    );

    joystick2.x().onTrue(
      Commands.runOnce(() -> {
        m_intake.setvolt(-7);
      }, m_intake)
    );
    
    joystick2.x().onFalse(
      Commands.runOnce(() -> {
        m_intake.setvolt(0.0);
      }, m_intake)
    );

    joystick2.y().onTrue(
      Commands.runOnce(() -> {
        m_intake.setvolt(7);
      }, m_intake)
    );
    
    joystick2.y().onFalse(
      Commands.runOnce(() -> {
        m_intake.setvolt(0);
      }, m_intake)
    );
  }
  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("BlueAmp1");
  }
}
