// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.autonomous.AutoRotate;
import frc.robot.subsystems.DriveSubsystem;


/**
 * Factories for autonomous commands
 * NOTE: Chuck will not use autonomous mode in the driverstation. Instead, all autonomous commands will be run in teleop mode
 */
public final class Autos {
    /** Example static factory for an autonomous command. */
    public static Command exampleAuto(ExampleSubsystem subsystem) {
        return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
