package com.stuypulse.robot.commands.arm;

import com.stuypulse.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class ArmMoveToAngle extends InstantCommand{
    private final Arm arm;
    private final double angle;

    public ArmMoveToAngle(double angle){
        arm = Arm.getInstance();
        this.angle = angle;
    }

    @Override
    public void initialize(){
        arm.setTargetAngle(angle);
    
    }
    
}