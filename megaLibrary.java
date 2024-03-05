// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.modules.KrakenSwerveModule;
import frc.robot.subsystems.DriveSubsystem.DriveConstants;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class megaLibrary {

    //MOTOR COMMANDS
    public void CANSetVolts(CANSparkMax motor, double volts){
        motor.setVoltage(volts);
    }

    public void FXSetVolts(TalonFX motor, double volts){
        motor.setVoltage(volts);
    }

    public void CANSet(CANSparkMax motor, double speed){
        motor.set(speed);
    }

    public void FXSet(TalonFX motor, double speed){
        motor.set(speed);
    }

    public void CANInvert(CANSparkMax motor, boolean invert){
        motor.setInverted(invert);
    }

    public void FXInvert(TalonFX motor, boolean invert){
        motor.setInverted(invert);
    }

    public void CANPID(CANSparkMax motor, double p, double i, double d){
        motor.getPIDController().setP(p);
        motor.getPIDController().setI(i);
        motor.getPIDController().setD(d);
    }

    public void FXSetPosition(TalonFX motor, double pos){
        motor.setPosition(pos);
    }

    public void CANSetPosition(CANSparkMax motor, double angle){
        motor.getEncoder().setPosition(angle);
    }

    public void CANFollow(CANSparkMax motor, CANSparkMax followMotor){
        motor.follow(followMotor, false);
    }

    public void FXFollow(TalonFX motor, Follower follower){
        follower = new Follower(motor.getDeviceID(), false);
    }

    public void CANGetVoltage(CANSparkMax motor){
        motor.getBusVoltage();
    }

    public void FXGetVoltage(TalonFX motor){
        motor.getMotorVoltage();
    }


    //LED COMMANDS
    public void initializeLED(int port, int length, AddressableLED led, AddressableLEDBuffer ledBuffer, int red, int green, int blue){
        led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);

        led.setLength(ledBuffer.getLength());
        led.setData(ledBuffer);

        for (int i = 0; i < ledBuffer.getLength(); i++)
        ledBuffer.setRGB(i, red, green, blue);

        led.start();
    }

    public void changeLEDColor(AddressableLED led, AddressableLEDBuffer ledBuffer, int red, int green, int blue){
        for (int i = 0; i < ledBuffer.getLength(); i++)
        ledBuffer.setRGB(i, red, green, blue);
        led.setData(ledBuffer);
    }

    public void sequenceLED(AddressableLEDBuffer ledBuffer, int red1, int green1, int blue1, int red2, int green2, int blue2){
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            if (i % 2 == 0)
              ledBuffer.setRGB(i, red1, green1, blue1);
            else
              ledBuffer.setRGB(i, red2, green2, blue2);
          }
    }

    //MATH STUFF

    public double estimateAngle(double height, double distance) {
        return Math.toDegrees(Math.atan(height / distance));
    }

    public double DriveConversionFactor(double wheelDiameter, double driveReduction){
        return Math.PI * wheelDiameter * driveReduction;
    }

    //ENCODERS

    public double angle(DutyCycleEncoder encoder) {
        return encoder.getAbsolutePosition() * 360;
    }

    //SWERVE STUFF
    public void initializeSwerveDriveKinematics(SwerveDriveKinematics kinematics){
            kinematics = new SwerveDriveKinematics(
            new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-DriveConstants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0,
                    -DriveConstants.DRIVETRAIN_WHEELBASE_METERS / 2.0));
    }
}
