package com.stuypulse.robot.commands.routines.funnel_side;

import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;
import com.stuypulse.robot.commands.arm.funnel_side.*;
import com.stuypulse.robot.commands.elevator.funnel_side.*;
import com.stuypulse.robot.commands.lokishooter.*;
import com.stuypulse.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MoveToL2Funnel extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    private LokiShooter shooter;
    public MoveToL2Funnel() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        shooter = LokiShooter.getInstance();
        addRequirements(arm, elevator, shooter);

        addCommands(
          new ElevatorToL2Funnel(),
          new ArmToL2Funnel(),
          new ShooterShootFunnel()
        );
    }
}