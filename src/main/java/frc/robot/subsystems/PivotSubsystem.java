// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later


package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class PivotSubsystem extends SubsystemBase {

    private final SparkFlex pivotMotor1;
    private final SparkFlex pivotMotor2;
    private final SmartMotorController motor;
    private final Pivot pivot;

    public PivotSubsystem() {

        // REV Spark Flex
        pivotMotor1 = new SparkFlex(Constants.PivotCanID1, MotorType.kBrushless);
        pivotMotor2 = new SparkFlex(Constants.PivotCanID2, MotorType.kBrushless);

        SparkFlexConfig followerConfig = new SparkFlexConfig();
        followerConfig.follow(pivotMotor1, true);
        pivotMotor2.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // YAMS motor configuration
        SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig()
                .withSubsystem(this)
                .withMotorInverted(false)
                .withStatorCurrentLimit(Amps.of(40))
                .withGearing(2.22)
                .withClosedLoopController(0,0,0)
                .withFeedforward(
                    new ArmFeedforward(0,0,0,0)
                )
                .withSoftLimits(
                    Degrees.of(-102),
                    Degrees.of(47)
                )
                .withStartingPosition(
                    Degrees.of(-102)
                )
                .withMomentOfInertia(
                    Meters.of(0.161),
                    Kilograms.of(30)
                )
                //.withMomentOfInertia(266.12108 + 6.077892e-4)
                .withIdleMode(
                    SmartMotorControllerConfig.MotorMode.BRAKE
                );

        // Wrap REV motor with YAMS
        motor = new SparkWrapper(
            pivotMotor1,
            DCMotor.getNeoVortex(2),
            motorConfig
        );

        // YAMS pivot configuration
        PivotConfig pivotConfig = new PivotConfig()
            .withHardLimits(
                Degrees.of(102),
                Degrees.of(-102)
            )
            .withTelemetry(
                "Pivot",
                SmartMotorControllerConfig.TelemetryVerbosity.HIGH
            );

        // Create pivot
        pivot = new Pivot(pivotConfig, motor);
    }

    // Move continuously to an angle
    public Command setAngle(Angle angle) {
        return pivot.setAngle(angle);
    }

    // Move to an angle with tolerance
    public Command moveTo(Angle angle) {
        return pivot.runTo(
            angle,
            Degrees.of(1)
        );
    }

    // Current angle
    public Angle getAngle() {
        return pivot.getAngle();
    }

    // Position check
    public boolean isNear(Angle angle) {
        return pivot.isNear(angle,Degrees.of(1)).getAsBoolean();
    }

    public void setSpeed(Supplier<Double> speed){
        pivotMotor1.set(speed.get());
    }

    @Override
    public void periodic() {
        pivot.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        pivot.simIterate();
    }

}