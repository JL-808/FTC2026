package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Globals {
    public static double PoseX = 0;
    public static double PoseY = 0;
    public static double PoseHeading = Math.toRadians(180);

    public static boolean isRed;

    public static final double FIRST_ROW_ARTIFACTS = 36;
    public static final double SECOND_ROW_ARTIFACTS = 60;
    public static final double THIRD_ROW_ARTIFACTS = 84;

    public static final double BEGIN_INTAKE = 45; // inches from wall to begin intake
    public static final double FIRST_ROW_STOP_INTAKE = 9; // inches from wall to stop intake
    public static final double SECOND_ROW_STOP_INTAKE = 10;

    public static final double THIRD_ROW_STOP_INTAKE = 16;

    public static final double FAR_LAUNCH_VELOCITY = 2080;
    public static final double SHORT_LAUNCH_VELOCITY = 2000;
}
