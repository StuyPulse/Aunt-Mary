package com.stuypulse.robot.commands.climb;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.climb.Climb;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimbDriveToAngle extends Command {
    private Climb climb;
    private final double targetDegrees;

    public ClimbDriveToAngle(double targetDegrees) {
        Climb climb = Climb.getInstance();
        addRequirements(climb);
        this.targetDegrees = targetDegrees;
    }

    public void initialize() {
        climb.setTargetDegrees(targetDegrees);
    }

    @Override
    public void end(boolean interrupted) {
        climb.stop();
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(climb.getDegrees() - climb.getTargetAngle().getDegrees()) <= Settings.Climb.CLIMB_ANGLE_TOLERANCE){
            return true;
        }
        else {
            return false;
        }
    }
}
