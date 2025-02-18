package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ElevatorOffsetTargetDown extends InstantCommand{
    private final Elevator elevator;

    public ElevatorOffsetTargetDown() {
        this.elevator = Elevator.getInstance();
    }

    @Override
    public void initialize() {
        elevator.setOperatorOffset(elevator.getOperatorOffset() - Settings.Operator.Elevator.HEIGHT_OFFSET_PER_CLICK);
    }
}
