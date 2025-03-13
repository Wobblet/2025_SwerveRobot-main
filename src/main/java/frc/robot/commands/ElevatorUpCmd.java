package frc.robot.commands;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants;

public class ElevatorUpCmd extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;
    //private final DigitalInput topLimitSwitch = new DigitalInput(0);
    
    public ElevatorUpCmd(ElevatorSubsystem elevatorSubsystem, double speed){
        this.elevatorSubsystem = elevatorSubsystem;
        this.speed = speed;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        elevatorSubsystem.setElevator(speed);
    }

    @Override
    public void end(boolean interrupted){
        elevatorSubsystem.setElevator(0);
    }

    @Override
    public boolean isFinished(){
        // if (elevatorSubsystem.getElevatorEncoderValue() >= Constants.NeoMotorConstants.elevatorTopLimit){
        //     return true;
        // }
        // else{
        //     return false;
        // }
        return false;
    }
}
