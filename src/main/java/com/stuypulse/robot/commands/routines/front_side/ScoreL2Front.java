package com.stuypulse.robot.commands.routines.front_side;

import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;
import com.stuypulse.robot.commands.arm.front_side.*;
import com.stuypulse.robot.commands.elevator.front_side.*;
import com.stuypulse.robot.commands.lokishooter.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreL2Front extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    private LokiShooter shooter;
    
    public ScoreL2Front() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        shooter = LokiShooter.getInstance();
        addRequirements(arm, elevator, shooter);

        addCommands(
          new ArmToL2Front(),
          new ElevatorToL2Front(),
          new ShooterShootFront()
        );
    }   
}