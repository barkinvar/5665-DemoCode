package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
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

    public Command scoreToReef(ElevatorTarget scoreHeight) {
        return mElevator.runToSetpoint(scoreHeight).andThen(mFunnel.shoot(3.5)).unless(mFunnel::getSensorState);
    }


}
