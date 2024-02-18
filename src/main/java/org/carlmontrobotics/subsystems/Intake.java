
package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.*;

import java.lang.annotation.Target;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.Robot;
import org.carlmontrobotics.RobotContainer;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;
import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

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

public class Intake extends SubsystemBase {
    CANSparkMax motor = MotorControllerFactory.createSparkMax(motorPort, MotorConfig.NEO_550); //double check motor port
    SparkPIDController pid = motor.getPIDController();
    RelativeEncoder motorEncoder = motor.getEncoder();
   

    private final MutableMeasure<Voltage> voltage = mutable(Volts.of(0));
    private final MutableMeasure<Velocity<Angle>> angularVel = mutable(RotationsPerSecond.of(0));
    private final MutableMeasure<Angle> distance = mutable(Rotations.of(0));
    private static double kS = Constants.kS;
    private static double kA = Constants.kA;
    private static double kV = Constants.kV;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV, kA); //double check constants | PLEASE TUNE IT TOMORROW TO FULLY TEST IT
    private TimeOfFlight distanceSensor = new TimeOfFlight(dsPort1); //make sure id port is correct here
    private TimeOfFlight distanceSenor2 = new TimeOfFlight(dsPort2); // insert 
    double kP = 0.00014312;
    double kD = 0;
    double kI = 0;
    private double dsDepth = 9.97;
    private double detectDistance = 13;
    public Intake() {
        // kinda no works. | motor.getEncoder().setPositionConversionFactor(2 * Math.PI * 2); //This will be different since the wheel diamters will not be the same. THIS IS FOR DISTANCE
        pid.setP(kP);
        pid.setD(kD);
        SmartDashboard.putNumber("kP" , kP);
        SmartDashboard.putNumber("kD", kD);
        SmartDashboard.putNumber("kI", kI);
        SmartDashboard.putNumber("kA" , kA);
        SmartDashboard.putNumber("kS", kS);
        SmartDashboard.putNumber("kV", kV);
        
        //SmartDashboard.putNumber("motorSpeed", 0); //Enter Positive Number for intake
        //SmartDashboard.putBoolean("useRPMSpeed", false);
        SmartDashboard.putNumber("Target RPM", 0);
    }
    public void driveMotor(Measure<Voltage> volts) {
        motor.setVoltage(volts.in(Volts));
    }
    public double getGamePieceDistance1() {
        return Units.metersToInches((distanceSensor.getRange() - dsDepth) /1000);
    }
    public double getGamePieceDistance2() {
        return Units.metersToInches((distanceSenor2.getRange() - dsDepth) /1000);
    }
    public boolean gameDistanceSees1st() {
        return getGamePieceDistance1() < detectDistance;
    }
    public boolean gameDistanceSees2nd() {
        return getGamePieceDistance2() < detectDistance;
    }
    @Override
    public void periodic() { 
        double TargetRPM = SmartDashboard.getNumber("Target RPM", 0);
        kP = SmartDashboard.getNumber("kP", kP);
        kD = SmartDashboard.getNumber("kD", kD);
        kI = SmartDashboard.getNumber("kI", kI);
        kA = SmartDashboard.getNumber("kA", kA);
        kS = SmartDashboard.getNumber("kS", kS);
        kV = SmartDashboard.getNumber("kV", kV);
        SmartDashboard.putBoolean("distance sensor 1", gameDistanceSees1st());
        SmartDashboard.putBoolean("distance sensor 2", gameDistanceSees2nd());
        SmartDashboard.putNumber("CurrentMotorRPM", motorEncoder.getVelocity()); //Enter Positive Number for intake
        if(pid.getP() != kP) {
            pid.setP(kP);
        }
        if (pid.getD() != kD) {
            pid.setD(kD);
        }
        if (pid.getI() != kI) {
            pid.setI(kI);
        }
        
        //pid.setReference((TargetRPM), CANSparkBase.ControlType.kVelocity,0, feedforward.calculate(TargetRPM/60));
        //TODO: Determine whether pid and feedforward values are accurate and why motor is constantly at 500 rpm
         SmartDashboard.putNumber("distance sensor 1",getGamePieceDistance1());
         SmartDashboard.putNumber("distance sensor 2",getGamePieceDistance2());

        if(!gameDistanceSees1st()) {
            pid.setReference((-TargetRPM), CANSparkBase.ControlType.kVelocity,0, feedforward.calculate(-TargetRPM/60));
         } else {    
            pid.setReference((-TargetRPM), CANSparkBase.ControlType.kVelocity,0,feedforward.calculate(-TargetRPM/240));
            if(gameDistanceSees2nd()) {
                pid.setReference(0, CANSparkBase.ControlType.kVelocity,0,feedforward.calculate(0));
            } 
        }
    }
    

    //Ahead are Sysid tests

    public void logMotor(SysIdRoutineLog log) {
        log.motor("intake-motor").voltage(voltage.mut_replace(
                motor.getBusVoltage() * motor.getAppliedOutput(),
                Volts)).angularVelocity(angularVel.mut_replace((motorEncoder.getVelocity()/60),
                        RotationsPerSecond))
                .angularPosition(distance.mut_replace((motor.getEncoder().getPosition()), Rotations));
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

}
