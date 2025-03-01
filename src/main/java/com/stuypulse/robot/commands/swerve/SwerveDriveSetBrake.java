package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwerveDriveSetBrake extends InstantCommand {
    private CommandSwerveDrivetrain swerve;

    public SwerveDriveSetBrake() {
        swerve = CommandSwerveDrivetrain.getInstance();
    }

    @Override
    public void initialize() {
        swerve.configNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
