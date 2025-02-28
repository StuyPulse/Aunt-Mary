package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveSetCoast extends InstantCommand {
    private CommandSwerveDrivetrain swerve;

    public SwerveDriveSetCoast() {
        swerve = CommandSwerveDrivetrain.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.configNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
