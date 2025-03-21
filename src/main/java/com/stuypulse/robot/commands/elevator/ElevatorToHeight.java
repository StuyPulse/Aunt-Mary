package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.subsystems.elevator.Elevator;
import com.stuypulse.robot.subsystems.elevator.Elevator.ElevatorState;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ElevatorToHeight extends InstantCommand{

    private final Elevator elevator;
    private final ElevatorState state;

    public ElevatorToHeight(ElevatorState state) {
        this.elevator = Elevator.getInstance();
        this.state = state;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setState(state);
    }
}
