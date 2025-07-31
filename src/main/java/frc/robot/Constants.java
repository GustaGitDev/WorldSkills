/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public final class Constants
{
    /**
     * Motor Constants
     */
    public static final int TITAN_ID        = 42; 
    public static final int M0           = 0;
    public static final int M1           = 1;
    public static final int M2           = 2;
    public static final double wheelRadius = 51;
    public static final double pulsePerRevolution = 1440;
    public static final double  gearRatio = 1/1;
    public static final double wheelPulseRatio = pulsePerRevolution * gearRatio;
    public static final double WHEEL_DIST_PER_TICK = (Math.PI * wheelRadius) /pulsePerRevolution;

}
