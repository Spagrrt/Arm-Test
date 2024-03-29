// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.util.Optional;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...

    ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    ElevatorManualCommand ankleCommand = new ElevatorManualCommand(elevatorSubsystem, ElevatorManualCommand.ArmControlMode.ANKLE);
    ElevatorManualCommand rotateCommand = new ElevatorManualCommand(elevatorSubsystem, ElevatorManualCommand.ArmControlMode.ROTATE);
    ElevatorManualCommand extendCommand = new ElevatorManualCommand(elevatorSubsystem, ElevatorManualCommand.ArmControlMode.EXTEND);

    ExtendToLengthCommand extendTo25 = new ExtendToLengthCommand(elevatorSubsystem, 0.55);
    RotateToAngleCommand rotateToAngleCommand = new RotateToAngleCommand(Math.toRadians(90), elevatorSubsystem);
    RotateAnkleCommand rotateAnkleCommand = new RotateAnkleCommand(Math.toRadians(80), elevatorSubsystem);

    ThreeAngleCommand pickupCommand = new ThreeAngleCommand(elevatorSubsystem, Optional.of(Math.toRadians(10)), Optional.of(0.185), Optional.of(Math.toRadians(-75)));
    ThreeAngleCommand speakerShootCommand = new ThreeAngleCommand(elevatorSubsystem, Optional.of(Math.toRadians(0)), Optional.of(0.0), Optional.of(Math.toRadians(50)));

    ShooterCommand powerCommand = new ShooterCommand(shooterSubsystem, ShooterCommand.ShooterType.POWER);
    ShooterCommand controlCommand = new ShooterCommand(shooterSubsystem, ShooterCommand.ShooterType.CONTROL);

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
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
    private void configureBindings()
    {
        //Constants.button5.whileTrue(ankleCommand);
        Constants.button3.whileTrue(pickupCommand);
        Constants.button4.whileTrue(speakerShootCommand);
        Constants.button6.whileTrue(controlCommand);
        //Constants.button7.whileTrue(rotateAnkleCommand);
        Constants.trigger.whileTrue(powerCommand);
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return null;
    }
}
