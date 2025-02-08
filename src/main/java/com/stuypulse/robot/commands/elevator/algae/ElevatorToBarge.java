package com.stuypulse.robot.commands.elevator.algae;

import com.stuypulse.robot.constants.Settings.Elevator;
import com.stuypulse.robot.commands.elevator.*;

public class ElevatorToBarge extends ElevatorToHeight{
    public ElevatorToBarge() {
        super(Elevator.BARGE_HEIGHT_METERS);
    }
}