package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Globals {
    public static double PoseX = 0;
    public static double PoseY = 0;
    public static double PoseHeading = Math.toRadians(180);

    public static boolean isRed;

    public static double FIRST_ROW_ARTIFACTS = 36.5;
    public static double SECOND_ROW_ARTIFACTS = 60.5;
    public static double THIRD_ROW_ARTIFACTS = 86;

    public static double BEGIN_INTAKE = 57; // inches from wall to begin intake
    public static double FIRST_ROW_STOP_INTAKE = 9.1; // inches from wall to stop intake
    public static double SECOND_ROW_STOP_INTAKE = 9.1;

    public static double THIRD_ROW_STOP_INTAKE = 16;

    public static double FAR_LAUNCH_VELOCITY = 2100;
    public static double SHORT_LAUNCH_VELOCITY = 1760;
}
