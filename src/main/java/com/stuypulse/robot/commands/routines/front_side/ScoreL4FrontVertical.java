package com.stuypulse.robot.commands.routines.front_side;

import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.lokishooter.LokiShooter;
import com.stuypulse.robot.commands.arm.ArmToVertical;
import com.stuypulse.robot.commands.arm.front_side.*;
import com.stuypulse.robot.commands.elevator.front_side.*;
import com.stuypulse.robot.commands.lokishooter.ShooterShootFront;
import com.stuypulse.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ScoreL4FrontVertical extends SequentialCommandGroup {
    private Arm arm;
    private Elevator elevator;
    private LokiShooter shooter;
    public ScoreL4FrontVertical() {
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        shooter = LokiShooter.getInstance();
        addRequirements(arm, elevator, shooter);

        addCommands(
          new ArmToVertical(),
          new ElevatorToL4Front(),
          // swerve align
          new ArmToL4FromVertical(),
          new ShooterShootFront()
        );
    }
}