package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FunnelSubsystem;
import frc.robot.Constants;

public class SetFunnelCmd extends Command {

    private final FunnelSubsystem funnelSubsystem;
    private final double speed;
    
    public SetFunnelCmd(FunnelSubsystem funnelSubsystem, double speed){
        this.funnelSubsystem = funnelSubsystem;
        this.speed = speed;
        addRequirements(funnelSubsystem);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        funnelSubsystem.setFunnel(speed);
    }

    @Override
    public void end(boolean interrupted){
        funnelSubsystem.setFunnel(0);
    }

    @Override
    public boolean isFinished(){
        // if (funnelSubsystem.getFunnelEncoderValue() >= Constants.NeoMotorConstants.funnelEncoderLimit){
        //     return true;
        // }
        // else{
        //     return false;
        // }
        return false;
    }
}
