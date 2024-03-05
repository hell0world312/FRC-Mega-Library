// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MegaLibrary;

/** Add your docs here. */
public class MegaMath {
    public double estimateAngle(double height, double distance) {
        return Math.toDegrees(Math.atan(height / distance));
    }

    public double DriveConversionFactor(double wheelDiameter, double driveReduction){
        return Math.PI * wheelDiameter * driveReduction;
    }
}
