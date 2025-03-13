package frc.robot.commands;

//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorDownCmd extends Command {

    private final ElevatorSubsystem elevatorSubsystem;
    private final double speed;
    //private final DigitalInput bottomLimitSwitch = new DigitalInput(1);
    
    public ElevatorDownCmd(ElevatorSubsystem elevatorSubsystem, double speed){
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
        // if (elevatorSubsystem.getElevatorEncoderValue() <= Constants.NeoMotorConstants.elevatorBottomLimit){
        //     return true;
        // }
        // else{
        //     return false;
        // }
        return false;
    }
}
