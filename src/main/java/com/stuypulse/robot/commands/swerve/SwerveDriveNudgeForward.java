package com.stuypulse.robot.commands.swerve;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDriveNudgeForward extends Command {
    private final CommandSwerveDrivetrain swerve;
    
    public SwerveDriveNudgeForward() {
        swerve = CommandSwerveDrivetrain.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        swerve.setControl(swerve.getRobotCentricSwerveRequest()
            .withVelocityX(Settings.Swerve.NUDGE_FORWARD_SPEED_METERS_PER_SECOND)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
    }

    @Override
    public void end(boolean interrupted) {
        swerve.setControl(swerve.getRobotCentricSwerveRequest()
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));
    }
    
}
