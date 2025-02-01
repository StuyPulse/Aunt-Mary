package com.stuypulse.robot.commands.elevator;

import com.stuypulse.robot.constants.Settings.Elevator;

public class ElevatorToHandoff extends ElevatorToHeight{
    public ElevatorToHandoff() {
        super(Elevator.HANDOFF_HEIGHT_METERS);
    }

    public void initialize() {
        super.initialize();
    }
}
