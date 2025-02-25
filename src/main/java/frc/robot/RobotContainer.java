// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralScoringCommand;
import frc.robot.commands.DumbAlgaeIntakeCommand;
import frc.robot.commands.DumbAlgaePivot;
import frc.robot.commands.SuperstructureMotorMove;
import frc.robot.commands.SwerveDriveTestCommand;
import frc.robot.subsystems.AlgaeHandlerSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorTelemetry;
import frc.robot.commands.KeepClosedCommand;
import frc.robot.commands.RobotAlignCommand;
import frc.robot.subsystems.ElevatorSubsystem;

import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.Level;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private SwerveDriveSubsystem swerveDriveSubsystem;
  private EndEffectorSubsystem endEffectorSubsystem = new EndEffectorSubsystem();
  private ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private AlgaeHandlerSubsystem algaeHandlerSubsystem = new AlgaeHandlerSubsystem();
  private SuperstructureSubsystem superstructureSubsystem = new SuperstructureSubsystem();

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
    private final XboxController driveController_HID = m_driverController.getHID();

//   private final CommandXboxController m_pitController = new CommandXboxController(OperatorConstants.kPitControllerPort);

  private final CommandGenericHID m_gunnerController = new CommandGenericHID(OperatorConstants.kGunnerControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    swerveDriveSubsystem = new SwerveDriveSubsystem();

    NamedCommands.registerCommand("L1", new CoralScoringCommand(endEffectorSubsystem, elevatorSubsystem));
    NamedCommands.registerCommand("Source",  new ParallelDeadlineGroup(
        new CoralIntakeCommand(endEffectorSubsystem),
        new KeepClosedCommand(superstructureSubsystem)));
    NamedCommands.registerCommand("L4 left",
        new SequentialCommandGroup(new ParallelCommandGroup(
          new ElevatorCommand(elevatorSubsystem, Level.L4, endEffectorSubsystem, false),
          new RobotAlignCommand(swerveDriveSubsystem, RobotAlignCommand.Mode.left, true),
          new WaitCommand(2)),
        new CoralScoringCommand(endEffectorSubsystem, elevatorSubsystem),
        new ElevatorCommand(elevatorSubsystem, Level.L1, endEffectorSubsystem, false)));

     NamedCommands.registerCommand("L4 right",
        new SequentialCommandGroup(new ParallelCommandGroup(
          new ElevatorCommand(elevatorSubsystem, Level.L4, endEffectorSubsystem, false),
          new RobotAlignCommand(swerveDriveSubsystem, RobotAlignCommand.Mode.right, true),
          new WaitCommand(2)),
        new CoralScoringCommand(endEffectorSubsystem, elevatorSubsystem),
        new ElevatorCommand(elevatorSubsystem, Level.L1, endEffectorSubsystem, false)));

      NamedCommands.registerCommand("L4", 
        new ElevatorCommand(elevatorSubsystem, Level.L4, endEffectorSubsystem, false)
      );
    
        NamedCommands.registerCommand("L2",
        new SequentialCommandGroup(new ElevatorCommand(elevatorSubsystem, Level.L2, endEffectorSubsystem, false),
            new WaitCommand(0.2), new CoralScoringCommand(endEffectorSubsystem, elevatorSubsystem)));
            

    swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
        () -> -MathUtil.applyDeadband(driveController_HID.getRawAxis(1), 0.1),
        () -> -MathUtil.applyDeadband(driveController_HID.getRawAxis(0), 0.1),
        () -> -MathUtil.applyDeadband(driveController_HID.getRawAxis(4), 0.1)

    ));

    // Configure the trigger bindings
    configureBindings();

    // Setup autoChooser
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  public void setupTelemetry() {
    ElevatorTelemetry elevatorTelemetry = new ElevatorTelemetry(elevatorSubsystem);

    elevatorTelemetry.schedule();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    m_driverController.rightTrigger().whileTrue(new CoralScoringCommand(endEffectorSubsystem, elevatorSubsystem));
    m_gunnerController.button(8).whileTrue(new CoralScoringCommand(endEffectorSubsystem, elevatorSubsystem));
    m_gunnerController.button(11).whileTrue(new ParallelCommandGroup(
        new SequentialCommandGroup(
            new ElevatorCommand(elevatorSubsystem, Level.LIntake, endEffectorSubsystem, false),
            new CoralIntakeCommand(endEffectorSubsystem),
            new ElevatorCommand(elevatorSubsystem, Level.L1, endEffectorSubsystem, false)),
        new KeepClosedCommand(superstructureSubsystem)));

    m_driverController.povDown().whileTrue(new SwerveDriveTestCommand(swerveDriveSubsystem, 0, -4, 0));
    m_driverController.povUp().whileTrue(new SwerveDriveTestCommand(swerveDriveSubsystem, 0, 4, 0));
    m_driverController.povRight().whileTrue(new SwerveDriveTestCommand(swerveDriveSubsystem, 0, 0, 4));
    m_driverController.povLeft().whileTrue(new SwerveDriveTestCommand(swerveDriveSubsystem, 0, 0, -4));
    // m_driverController.povDown().onTrue(new
    // SuperstructureEncoderResetCommand(superstructureSubsystem));
    // m_driverController.leftTrigger().whileTrue(new
    // SuperstructureMotorMove(superstructureSubsystem,superstructureSubsystem.getLeftMotor(),0.2));
    // m_driverController.rightTrigger().whileTrue(new
    // SuperstructureMotorMove(superstructureSubsystem,superstructureSubsystem.getRightMotor(),-0.2));
    // m_driverController.leftBumper().whileTrue(new
    // SuperstructureMotorMove(superstructureSubsystem,superstructureSubsystem.getLeftMotor(),-0.2));
    m_driverController.rightBumper()
        .whileTrue(new SuperstructureMotorMove(superstructureSubsystem, superstructureSubsystem.getRightMotor(), 0.2));

    m_driverController.y().whileTrue(new RobotAlignCommand(
      swerveDriveSubsystem,
      () -> -MathUtil.applyDeadband(driveController_HID.getRawAxis(0), 0.1),
      RobotAlignCommand.Mode.manual
    ));
    m_driverController.x().whileTrue(new RobotAlignCommand(swerveDriveSubsystem, RobotAlignCommand.Mode.left, false));
    m_driverController.b().whileTrue(new RobotAlignCommand(swerveDriveSubsystem, RobotAlignCommand.Mode.right, false));

    m_driverController.leftBumper().onTrue(new InstantCommand(() -> swerveDriveSubsystem.resetGyro()));

    m_driverController.a().onTrue(new InstantCommand(() -> algaeHandlerSubsystem.resetEncoder()));

    // m_gunnerController.button(5).onTrue(new VerticleClimberCommand(climberSubsystem));
    // m_gunnerController.button(2).onTrue(new AngledClimberCommand(climberSubsystem));

    m_gunnerController.button(2).whileTrue(new ClimberCommand(climberSubsystem, ClimberSubsystem.Stage.S1));
    m_gunnerController.button(5).whileTrue(new ClimberCommand(climberSubsystem, ClimberSubsystem.Stage.S2));

    // m_gunnerController.button(2).onTrue(new InstantCommand(() ->
    // climberSubsystem.dumbClimbComp()));
    // m_gunnerController.button(2).onFalse(new InstantCommand(() ->
    // climberSubsystem.climberStop()));

    m_gunnerController.button(1).onTrue(new SequentialCommandGroup(
        new SuperstructureMotorMove(superstructureSubsystem, superstructureSubsystem.getLeftMotor(), -0.2),
        new SuperstructureMotorMove(superstructureSubsystem, superstructureSubsystem.getRightMotor(), -0.2),
        new WaitCommand(0.5),
        new SuperstructureMotorMove(superstructureSubsystem, superstructureSubsystem.getLeftMotor(), 0.2),
        new WaitCommand(1.5),
        new SuperstructureMotorMove(superstructureSubsystem, superstructureSubsystem.getLeftMotor(), 0),
        new SuperstructureMotorMove(superstructureSubsystem, superstructureSubsystem.getRightMotor(), 0)));

    m_gunnerController.button(4).whileTrue(new DumbAlgaePivot(algaeHandlerSubsystem, 0.1));
    m_gunnerController.button(7).whileTrue(new DumbAlgaeIntakeCommand(algaeHandlerSubsystem));
    m_gunnerController.button(10).whileTrue(new DumbAlgaePivot(algaeHandlerSubsystem, -0.1));

    // m_gunnerController.button(7).whileTrue(new
    // AlgaePivotCommand(algaeHandlerSubsystem, algaeHandlerSubsystem));

    m_gunnerController.button(12).onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.L1,
        endEffectorSubsystem, algaeHandlerSubsystem.isIntakingAlgae()));
    m_gunnerController.button(9)
        .onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.L2, endEffectorSubsystem, false));
    m_gunnerController.button(6)
        .onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.L3, endEffectorSubsystem, false));
    m_gunnerController.button(3)
        .onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.L4, endEffectorSubsystem, false));

    // m_driverController.rightTrigger().whileTrue(new
    // DumbElevatorCommand(elevatorSubsystem, true));
    // m_driverController.leftTrigger().whileTrue(new
    // DumbElevatorCommand(elevatorSubsystem, false));

    // m_pitController.a().onTrue(new InstantCommand(() -> climberSubsystem.dumbClimbComp()));
    // m_pitController.a().onFalse(new InstantCommand(() -> climberSubsystem.climberStop()));
    // m_pitController.b().onTrue(new InstantCommand(() -> climberSubsystem.dumbClimb()));
    // m_pitController.b().onFalse(new InstantCommand(() -> climberSubsystem.climberStop()));
    // m_pitController.x().onTrue(new InstantCommand(() -> climberSubsystem.resetEncoder()));

    // m_pitController.povDown().onTrue(new InstantCommand(() -> algaeHandlerSubsystem.pivotDown()));
    // m_pitController.povUp().onTrue(new InstantCommand(() -> algaeHandlerSubsystem.pivotUp()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
