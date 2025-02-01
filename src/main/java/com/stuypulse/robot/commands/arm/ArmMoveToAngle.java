package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class ArmMoveToAngle extends InstantCommand{
    private final Arm arm;
    private final Rotation2d angle;

    public ArmMoveToAngle(Rotation2d angle){
        arm = Arm.getInstance();
        this.angle = angle;
    }

    @Override
    public void initialize(){
        arm.setTargetAngle(angle);
    
    }
    
}