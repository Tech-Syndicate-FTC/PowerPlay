package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState;

public class ElevatorHomeCommand extends CommandBase {
    Elevator elevator;

    public ElevatorHomeCommand(Elevator elevator) {
        this.elevator = elevator;
    }

    @Override
    public void initialize() {
        elevator.homeElevator();
    }

    @Override
    public boolean isFinished() {
        return elevator.getState() != ElevatorState.HOMING;
    }
}
