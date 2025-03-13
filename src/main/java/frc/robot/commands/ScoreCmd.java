package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class ScoreCmd extends Command {

    private final EndEffectorSubsystem endEffectorSubsystem;
    private final double speed;
    
    public ScoreCmd(EndEffectorSubsystem endEffectorSubsystem, double speed){
        this.endEffectorSubsystem = endEffectorSubsystem;
        this.speed = speed;
        addRequirements(endEffectorSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        endEffectorSubsystem.setEndEffector(speed);
    }

    @Override
    public void end(boolean interrupted){
        endEffectorSubsystem.setEndEffector(0);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
