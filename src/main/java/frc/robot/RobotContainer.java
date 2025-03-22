// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeIntakeCommandGroup;
import frc.robot.commands.AlgaeLaunchCommand;
import frc.robot.commands.ClimberCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralScoringCommand;
import frc.robot.commands.DumbAlgaeIntakeCommand;
import frc.robot.subsystems.AlgaeHandlerSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.KeepClosedCommand;
import frc.robot.commands.OpenDoorCommand;
import frc.robot.commands.align.RobotAlignCommand;
import frc.robot.commands.align.Constants.Mode;
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
        private final CommandGenericHID m_gunnerController = new CommandGenericHID(
                        OperatorConstants.kGunnerControllerPort);

        private final CommandXboxController m_XboxController = new CommandXboxController(
                        OperatorConstants.kXBoxControllerPort);

        private final XboxController xboxController_HID = m_XboxController.getHID();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {

                swerveDriveSubsystem = new SwerveDriveSubsystem();

                NamedCommands.registerCommand("L1", new CoralScoringCommand(endEffectorSubsystem, elevatorSubsystem));
                NamedCommands.registerCommand("Source", new ParallelDeadlineGroup(
                                new CoralIntakeCommand(endEffectorSubsystem),
                                new KeepClosedCommand(superstructureSubsystem)));
                NamedCommands.registerCommand("L4 left",
                                new SequentialCommandGroup(new ParallelCommandGroup(
                                                new ElevatorCommand(elevatorSubsystem, Level.L4),
                                                new RobotAlignCommand(swerveDriveSubsystem, Mode.LEFT, true)),
                                                new CoralScoringCommand(endEffectorSubsystem, elevatorSubsystem),
                                                new ElevatorCommand(elevatorSubsystem, Level.LIntake)));

                NamedCommands.registerCommand("L4 right",
                                new SequentialCommandGroup(new ParallelCommandGroup(
                                                new ElevatorCommand(elevatorSubsystem, Level.L4),
                                                new RobotAlignCommand(swerveDriveSubsystem, Mode.RIGHT, true)),
                                                new CoralScoringCommand(endEffectorSubsystem, elevatorSubsystem),
                                                new ElevatorCommand(elevatorSubsystem, Level.LIntake)));

                NamedCommands.registerCommand("L4",
                                new ElevatorCommand(elevatorSubsystem, Level.L4));

                NamedCommands.registerCommand("L2",
                                new SequentialCommandGroup(
                                                new ElevatorCommand(elevatorSubsystem, Level.L2),
                                                new WaitCommand(0.2),
                                                new CoralScoringCommand(endEffectorSubsystem, elevatorSubsystem)));

                if (!OperatorConstants.usingXBox) {
                        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
                                        () -> -MathUtil.applyDeadband(driveController_HID.getRawAxis(3), 0.1),
                                        () -> -MathUtil.applyDeadband(driveController_HID.getRawAxis(1), 0.1),
                                        () -> -MathUtil.applyDeadband(driveController_HID.getRawAxis(0), 0.1)));
                } else {
                        swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
                                        () -> -MathUtil.applyDeadband(xboxController_HID.getRawAxis(1), 0.1),
                                        () -> -MathUtil.applyDeadband(xboxController_HID.getRawAxis(0), 0.1),
                                        () -> -MathUtil.applyDeadband(xboxController_HID.getRawAxis(4), 0.1)));
                }

                // Configure the trigger bindings
                configureBindings();

                // Setup autoChooser
                autoChooser = AutoBuilder.buildAutoChooser();
                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        public void reset() {
                this.algaeHandlerSubsystem.reset();
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
                var scoreCommand = new CoralScoringCommand(endEffectorSubsystem, elevatorSubsystem);
                var intakeCommand = new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                                new ElevatorCommand(elevatorSubsystem, Level.LIntake),
                                                new CoralIntakeCommand(endEffectorSubsystem)),
                                new KeepClosedCommand(superstructureSubsystem));

                m_XboxController.rightTrigger().whileTrue(scoreCommand);
                m_XboxController.x().whileTrue(new RobotAlignCommand(swerveDriveSubsystem, Mode.LEFT));
                m_XboxController.b().whileTrue(new RobotAlignCommand(swerveDriveSubsystem, Mode.RIGHT));

                m_XboxController.leftBumper().onTrue(new InstantCommand(() -> swerveDriveSubsystem.resetGyro()));
                m_XboxController.povUp().onTrue(new InstantCommand(() -> climberSubsystem.dumbClimbComp()));
                m_XboxController.povUp().onFalse(new InstantCommand(() -> climberSubsystem.climberStop()));

                m_XboxController.povDown().onTrue(new InstantCommand(() -> climberSubsystem.dumbClimb()));
                m_XboxController.povDown().onFalse(new InstantCommand(() -> climberSubsystem.climberStop()));

                m_driverController.button(1).whileTrue(new RobotAlignCommand(swerveDriveSubsystem, Mode.LEFT));
                m_driverController.button(2).whileTrue(scoreCommand);
                m_driverController.button(3).onTrue(new InstantCommand(() -> swerveDriveSubsystem.resetGyro()));
                m_driverController.button(4).onTrue(new InstantCommand(() -> climberSubsystem.dumbClimbComp()));
                m_driverController.button(4).onFalse(new InstantCommand(() -> climberSubsystem.climberStop()));
                m_gunnerController.button(8)
                                .whileTrue(new CoralScoringCommand(endEffectorSubsystem, elevatorSubsystem));
                m_gunnerController.button(11).whileTrue(intakeCommand);
                m_gunnerController.button(2)
                                .whileTrue(new ClimberCommand(climberSubsystem, ClimberSubsystem.Stage.S1,
                                                superstructureSubsystem));
                m_gunnerController.button(5)
                                .whileTrue(new ClimberCommand(climberSubsystem, ClimberSubsystem.Stage.S2,
                                                superstructureSubsystem));
                m_gunnerController.button(1).whileTrue(new OpenDoorCommand(superstructureSubsystem));
                m_gunnerController.button(4)
                                .whileTrue(new AlgaeIntakeCommandGroup(algaeHandlerSubsystem, elevatorSubsystem));
                m_gunnerController.button(7)
                                .whileTrue(new DumbAlgaeIntakeCommand(algaeHandlerSubsystem, elevatorSubsystem));
                m_gunnerController.button(7).onFalse(new SequentialCommandGroup(new WaitCommand(0.2),
                                new InstantCommand(() -> algaeHandlerSubsystem.stopIntake())));
                m_gunnerController.button(10).whileTrue(
                                new AlgaeLaunchCommand(algaeHandlerSubsystem));
                m_gunnerController.button(12)
                                .onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.L1));
                m_gunnerController.button(9)
                                .onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.L2));
                m_gunnerController.button(6)
                                .onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.L3));
                m_gunnerController.button(3)
                                .onTrue(new ElevatorCommand(elevatorSubsystem, ElevatorSubsystem.Level.L4));
        }

        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }
}
