package com.stuypulse.robot.commands.routines.funnel_side;

import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;
import com.stuypulse.robot.commands.arm.funnel_side.*;
import com.stuypulse.robot.commands.elevator.funnel_side.*;
import com.stuypulse.robot.commands.lokishooter.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveToL3Funnel extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    private LokiShooter shooter;
    public MoveToL3Funnel() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        addRequirements(arm, elevator, shooter);

        addCommands(
          new ElevatorToL3Funnel(),
          new ArmToL3Funnel(),
          new ShooterShootFunnel()
        );
    }
}