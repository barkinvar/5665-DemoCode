package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem.ElevatorTarget;
import frc.robot.subsystems.funnel.FunnelSubsystem;

public class Superstructure {
    
    private ElevatorSubsystem mElevator;
    private FunnelSubsystem mFunnel;

    public Superstructure(ElevatorSubsystem elevator, FunnelSubsystem funnel){
        mElevator = elevator;
        mFunnel = funnel;
    }

    public Command scoreToReef(ElevatorTarget scoreHeight, BooleanSupplier trigger) {
        return (mElevator.runToSetpoint(scoreHeight).andThen(Commands.waitUntil(trigger), mFunnel.shoot(3.5).withTimeout(0.25), mElevator.setForgetSetpointCommand(ElevatorTarget.HOME))).unless(() -> !mFunnel.getSensorState());
    }
}
