package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class PivotCmd extends Command{
    public PivotSubsystem pivotSubsystem;
    public double degrees;

    public PivotCmd(PivotSubsystem pivotSubsystem, double degrees){
        this.pivotSubsystem = pivotSubsystem;
        this.degrees = degrees;
        addRequirements(pivotSubsystem);
    }

    @Override
    public void execute(){
        //This gets called when the command does.
        pivotSubsystem.moveTo(Degrees.of(degrees));
        
    }

    @Override
    public void end(boolean interrupted){
        //This gets called when the input stops being given.
    }
    @Override
    public boolean isFinished() {
        return false;
    } 

}
