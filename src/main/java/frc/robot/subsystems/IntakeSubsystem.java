package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{

    private final SparkFlex intakeMotor;
    public IntakeSubsystem(){
        intakeMotor = new SparkFlex(Constants.intakeCanID, MotorType.kBrushless);
    }
    public void intake(double voltage) {
        intakeMotor.set(voltage);
    }

    public void stopRoller() {
        intakeMotor.stopMotor();
    }

    @Override
    public void periodic() {

    }


}
