package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.Constants;

public class ClimbCmd extends Command {

    private final ClimberSubsystem climberSubsystem;
    private final double speed;
    
    public ClimbCmd(ClimberSubsystem climberSubsystem, double speed){
        this.climberSubsystem = climberSubsystem;
        this.speed = speed;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        climberSubsystem.setClimber(speed);
    }

    @Override
    public void end(boolean interrupted){
        climberSubsystem.setClimber(0);
    }

    @Override
    public boolean isFinished(){
        // if (climberSubsystem.getClimberEncoderValue() >= Constants.NeoMotorConstants.climberEncoderLimit){
        //     return true;
        // }
        // else{
        //     return false;
        // }
        return false;
    }
}
