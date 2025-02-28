package com.stuypulse.robot.commands.vision;


import com.stuypulse.robot.subsystems.vision.LimelightVision;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class VisionSetWhiteList extends InstantCommand {

    private final LimelightVision vision;
    private final int[] ids;

    public VisionSetWhiteList(int... ids) {
        this.vision = LimelightVision.getInstance();
        this.ids = ids;
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        vision.setTagWhitelist(ids);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
