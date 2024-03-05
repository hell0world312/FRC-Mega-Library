// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MegaLibrary;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class TalonFXControls {
        public void FXSetVolts(TalonFX motor, double volts){
        motor.setVoltage(volts);
    }

    public void FXSet(TalonFX motor, double speed){
        motor.set(speed);
    }

    public void FXInvert(TalonFX motor, boolean invert){
        motor.setInverted(invert);
    }

    public void FXSetPosition(TalonFX motor, double pos){
        motor.setPosition(pos);
    }

    public void FXFollow(TalonFX motor, Follower follower){
        follower = new Follower(motor.getDeviceID(), false);
    }

    public void FXGetVoltage(TalonFX motor){
        motor.getMotorVoltage();
    }
}
