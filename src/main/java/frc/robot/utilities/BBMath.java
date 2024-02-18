// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

/** Add your docs here. */
public class BBMath {
public BBMath() {}

public static double getRps(double dps, double wheelDiameter) {
	return dps / (java.lang.Math.PI * wheelDiameter);
}
}
