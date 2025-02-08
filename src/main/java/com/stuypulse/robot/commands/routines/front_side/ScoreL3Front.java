package com.stuypulse.robot.commands.routines.front_side;

import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;
import com.stuypulse.robot.subsystems.arm.Arm;
import com.stuypulse.robot.commands.arm.front_side.*;
import com.stuypulse.robot.commands.elevator.front_side.*;
import com.stuypulse.robot.commands.lokishooter.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreL3Front extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    private LokiShooter shooter;
    public ScoreL3Front() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        shooter = LokiShooter.getInstance();
        addRequirements(arm, elevator, shooter);

        addCommands(
          new ArmToL3Front(),
          new ElevatorToL3Front(),
          new ShooterShootFront()
        );
    }   
}