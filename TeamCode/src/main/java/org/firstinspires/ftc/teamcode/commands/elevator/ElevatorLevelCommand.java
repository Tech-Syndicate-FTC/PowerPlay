package org.firstinspires.ftc.teamcode.commands.elevator;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator.Junctions;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorState;

public class ElevatorLevelCommand extends CommandBase {
    Elevator elevator;
    Junctions targetJunction;
    int allowedError = 20;

    public ElevatorLevelCommand(Elevator elevator, Junctions junctions) {
        this.elevator = elevator;
        targetJunction = junctions;
    }

    public ElevatorLevelCommand(Elevator elevator, Junctions junctions, int allowedError) {
        this.elevator = elevator;
        targetJunction = junctions;
        this.allowedError = allowedError;
    }

    @Override
    public void initialize() {
        elevator.enableLift();
        elevator.setLevel(targetJunction);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(elevator.getLiftRawPosition() - elevator.getLiftTargetPosition()) > allowedError;
    }
}
