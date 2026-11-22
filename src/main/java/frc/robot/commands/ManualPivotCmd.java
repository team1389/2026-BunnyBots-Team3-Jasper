package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class ManualPivotCmd extends Command{
    public PivotSubsystem pivotSubsystem;
    public Supplier<Double> speed;

    public ManualPivotCmd(PivotSubsystem pivotSubsystem, Supplier<Double> speed){
        this.pivotSubsystem = pivotSubsystem;
        this.speed = speed;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute(){
        //This gets called when the command does.
        pivotSubsystem.setSpeed(speed);
        
    }

    @Override
    public void end(boolean interrupted){
        //This gets called when the input stops being given.
        pivotSubsystem.setSpeed(() -> 0.0);
    }
    @Override
    public boolean isFinished() {
        return false;
    } 

}