package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveXMode extends Command {
    
    private final CommandSwerveDrivetrain swerve;

    public SwerveDriveXMode() {
        swerve = CommandSwerveDrivetrain.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.setControl(new SwerveRequest.SwerveDriveBrake());
    }

}
