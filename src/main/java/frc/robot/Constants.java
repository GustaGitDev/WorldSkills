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
    public static final int M3           = 3;
    public static final int DIF_SERVO = 0;
    
    public static final double wheelRadius = 51;
    public static final double pulsePerRevolution = 1440;
    public static final double  gearRatio = 1/1;
    public static final double wheelPulseRatio = pulsePerRevolution * gearRatio;
    public static final double WHEEL_DIST_PER_TICK = (Math.PI * wheelRadius) /pulsePerRevolution;
    //elevator de correia (se for o caso)
    public static final double pulleyRadius = 7.85;
    public static final double pulsePerRevElevator = 1440;
    public static final double elevatorGearRatio = 1.0/2.0;
    public static final double pulleyPulseRatio = pulsePerRevElevator * elevatorGearRatio;
    public static final double ELEVATOR_DIST_TICK = (Math.PI * 2 * pulleyRadius)/pulleyPulseRatio;
}
