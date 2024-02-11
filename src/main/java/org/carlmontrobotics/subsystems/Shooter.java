package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.*;

import org.carlmontrobotics.Constants.SHOOTER;
import org.carlmontrobotics.Robot;
import org.carlmontrobotics.RobotContainer;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Shooter extends SubsystemBase {
    CANSparkMax motor = MotorControllerFactory.createSparkMax(1, MotorConfig.NEO_550);
    SparkPIDController pidController = motor.getPIDController();
    private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));
    private final MutableMeasure<Angle> distance = mutable(Rotations.of(0));
    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(SHOOTER.kS, SHOOTER.kV, SHOOTER.kA);
    double kP = 0.0001;
    double kD = 0;
    double kI = 0;
    double kIZone = 0;
    RelativeEncoder encoder = motor.getEncoder();

    public Shooter() {
        pidController.setP(SHOOTER.kP);
        pidController.setD(SHOOTER.kD);
        pidController.setI(kI);
        SmartDashboard.putNumber("Shooter RPS", 0);
        SmartDashboard.putNumber("kP", kP);
        SmartDashboard.putNumber("kI", kP);
        // SmartDashboard.putNumber("kIZone", kIZone);
        SmartDashboard.putNumber("kD", kD);
        SmartDashboard.putBoolean("Use FF", true);
        SmartDashboard.putNumber("Feedforward", 0);
        // SmartDashboard.putNumber("plswork", 0);
        // THIS LINE BELOW DOESN'T WORK
        // encoder.setVelocityConversionFactor(1/9999999);
    }

    public void driveMotor(Measure<Voltage> volts) {
        motor.setVoltage(volts.in(Volts));
    }

    public void logMotor(SysIdRoutineLog log) {
        log.motor("shooter-motor")
                .voltage(voltage.mut_replace(
                        motor.getBusVoltage() * motor.getAppliedOutput(),
                        Volts))
                .angularVelocity(velocity.mut_replace(
                        encoder.getVelocity() / 60,
                        RotationsPerSecond))
                .angularPosition(distance.mut_replace(
                        encoder.getPosition(),
                        Rotations));
    }

    private final SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                    this::driveMotor,
                    this::logMotor,
                    this));

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    @Override
    public void periodic() {
        double targetRPS = SmartDashboard.getNumber("Shooter RPS", 0);
        kP = SmartDashboard.getNumber("kP", kP);
        kD = SmartDashboard.getNumber("kD", kD);
        //kIZone = SmartDashboard.getNumber("kIZone", kIZone);
        kI = SmartDashboard.getNumber("kI", kI);

        if (pidController.getP() != kP) {
            pidController.setP(kP);
        }
        if (pidController.getD() != kD) {
            pidController.setD(kD);
        }
        if (pidController.getI() != kI) {
            pidController.setI(kI);
        }
        // if (pidController.getIZone() != kIZone) {
        //     pidController.setIZone(kIZone);
        // }
        // motor.setVoltage(SmartDashboard.getNumber("plswork", 0));

        SmartDashboard.putNumber("Shooter current RPS", encoder.getVelocity() / 60);
        double feed = ff.calculate(targetRPS);

        if (SmartDashboard.getBoolean("Use FF", true)) {
            feed = SmartDashboard.getNumber("Feedforward", feed);
        } else {
            SmartDashboard.putNumber("Feedforward", feed);
        }
        SmartDashboard.putNumber("Error", motor.getBusVoltage() * motor.getAppliedOutput() - feed);
        SmartDashboard.putNumber("Motor Voltage", motor.getBusVoltage() * motor.getAppliedOutput());

        // motor.setVoltage(feed);
        pidController.setReference(targetRPS * 60, CANSparkBase.ControlType.kVelocity, 0, feed);
    }

}
