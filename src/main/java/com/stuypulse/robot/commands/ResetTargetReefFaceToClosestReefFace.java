package com.stuypulse.robot.commands;

import com.stuypulse.robot.util.ReefUtil;
import com.stuypulse.robot.util.TargetReefFaceManager;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ResetTargetReefFaceToClosestReefFace extends InstantCommand {
    public ResetTargetReefFaceToClosestReefFace() {
        super(() -> TargetReefFaceManager.reset(ReefUtil.getClosestReefFace()));
    }
}
