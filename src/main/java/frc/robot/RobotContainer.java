// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AngledClimberCommand;
import frc.robot.commands.ClimberTelemetry;
import frc.robot.commands.EndEffectorCommand;
import frc.robot.commands.VerticleClimberCommand;
import frc.robot.subsystems.AlgaeHandlerSubsystem;
import frc.robot.subsystems.ClimberSubsystem;

import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.ElevatorTelemetry;
import frc.robot.commands.EndEffector2Command;
import frc.robot.subsystems.ElevatorSubsystem;

import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
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
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  private final CommandXboxController m_pitController =
      new CommandXboxController(OperatorConstants.kPitControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    swerveDriveSubsystem = new SwerveDriveSubsystem();

    //NamedCommands.registerCommands("L1", endEffectorSubsystem.());


    swerveDriveSubsystem.setDefaultCommand(swerveDriveSubsystem.driveCommand(
      ()-> -MathUtil.applyDeadband(m_driverController.getRawAxis(1), 0.1),
      ()-> -MathUtil.applyDeadband(m_driverController.getRawAxis(0), 0.1),
      ()-> -MathUtil.applyDeadband(m_driverController.getRawAxis(4), 0.1)
      

    ));

    // Configure the trigger bindings
    configureBindings();
    


    // Setup autoChooser
     autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    

    
  }

  public void setupTelemetry(){
    ElevatorTelemetry elevatorTelemetry = new ElevatorTelemetry(elevatorSubsystem);
    ClimberTelemetry climberTelemetry = new ClimberTelemetry(climberSubsystem);
   elevatorTelemetry.schedule();
   climberTelemetry.schedule();
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
   
 //  m_driverController.a().whileTrue(new EndEffectorCommand(endEffectorSubsystem));
 //  m_driverController.b().whileTrue(new EndEffector2Command(endEffectorSubsystem));

   m_driverController.b().onTrue(new VerticleClimberCommand(climberSubsystem));
   m_driverController.x().onTrue(new AngledClimberCommand(climberSubsystem));

    m_driverController.povDown().onTrue(new ElevatorCommand(elevatorSubsystem,ElevatorSubsystem.Level.L1 ));
    m_driverController.povLeft().onTrue(new ElevatorCommand(elevatorSubsystem,ElevatorSubsystem.Level.L2 ));
    m_driverController.povUp().onTrue(new ElevatorCommand(elevatorSubsystem,ElevatorSubsystem.Level.L3 ));
    m_driverController.povRight().onTrue(new ElevatorCommand(elevatorSubsystem,ElevatorSubsystem.Level.L4 ));
    
    m_driverController.rightBumper().whileTrue(new AlgaeIntakeCommand(algaeHandlerSubsystem));

    m_pitController.y().onTrue(new InstantCommand(() -> climberSubsystem.climberUp()));
    m_pitController.y().onFalse(new InstantCommand(() -> climberSubsystem.climberStop()));
    m_pitController.a().onTrue(new InstantCommand(() -> climberSubsystem.climberDown()));
    m_pitController.a().onFalse(new InstantCommand(() -> climberSubsystem.climberStop()));
    m_pitController.x().onTrue(new InstantCommand(() -> climberSubsystem.resetEncoder()));
   
  }


    public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
