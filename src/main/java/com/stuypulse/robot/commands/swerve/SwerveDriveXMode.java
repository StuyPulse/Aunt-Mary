package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.PointWheelsAt;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveXMode extends Command {
    public SwerveDriveXMode() {
        SwerveRequest request = new SwerveRequest.SwerveDriveBrake();
        CommandSwerveDrivetrain.getInstance().setControl(request);
    }
}
