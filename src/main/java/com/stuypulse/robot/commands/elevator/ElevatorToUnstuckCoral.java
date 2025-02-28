package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorToUnstuckCoral extends Command {
    private Elevator elevator;

    public ElevatorToUnstuckCoral() {
        elevator = Elevator.getInstance();
        addRequirements(elevator);
    }

    public void initialize() {
        elevator.setState(ElevatorState.UNSTUCK_CORAL);
    }
}
