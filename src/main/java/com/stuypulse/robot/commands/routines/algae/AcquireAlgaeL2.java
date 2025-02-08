package com.stuypulse.robot.commands.routines.algae;

import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;
import com.stuypulse.robot.commands.arm.algae.*;
import com.stuypulse.robot.commands.elevator.algae.*;
import com.stuypulse.robot.commands.lokishooter.*; 
import com.stuypulse.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AcquireAlgaeL2 extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    private LokiShooter shooter;
    public AcquireAlgaeL2() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        shooter = LokiShooter.getInstance();
        addRequirements(arm, elevator, shooter);

        addCommands(
          new ElevatorToAlgaeL2(),
          new ArmToAlgaeL2(),
          new ShooterAcquireAlgae()
        );
        
    }
    
} 