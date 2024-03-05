// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MegaLibrary;
import com.revrobotics.CANSparkMax;

/** Add your docs here. */
public class CANSpark {
    public void SetVolts(CANSparkMax motor, double volts){
        motor.setVoltage(volts);
    }

    public void SetSpeed(CANSparkMax motor, double speed){
        motor.set(speed);
    }

    public void Invert(CANSparkMax motor, boolean invert){
        motor.setInverted(invert);
    }

    public void SetPID(CANSparkMax motor, double p, double i, double d){
        motor.getPIDController().setP(p);
        motor.getPIDController().setI(i);
        motor.getPIDController().setD(d);
    }

    public void SetPosition(CANSparkMax motor, double angle){
        motor.getEncoder().setPosition(angle);
    }

    public void Follow(CANSparkMax motor, CANSparkMax followMotor){
        motor.follow(followMotor, false);
    }

    public void GetVoltage(CANSparkMax motor){
        motor.getBusVoltage();
    }
}
