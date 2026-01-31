package org.firstinspires.ftc.teamcode;
import com.bylazar.configurables.annotations.Configurable;

import java.lang.Math;

@Configurable
public class LaunchCalculator {
    public static double launchOffset = 6.25;
    public static double offsetDegrees = 0;
    public static double heading(double x, double y, boolean isRed) {
        // https://www.desmos.com/calculator/kzbkneoyfr

        double m;
        double n;

        if (isRed) {
            double d = Math.sqrt(Math.pow(144 - x, 2) + Math.pow(144 - y, 2));
            if (d < launchOffset) {
                return 0;
            }
            m = Math.acos(launchOffset / d);
            n = Math.atan2(144 - y, 144 - x);

        } else {
            double d = Math.sqrt(Math.pow(-x, 2) + Math.pow(144 - y, 2));
            if (d <= launchOffset) {
                return 0;
            }
            m = Math.acos(launchOffset / d);
            n = Math.atan2(144 - y, -x);

        }
        return Math.PI - m + n + offsetDegrees/180*Math.PI;
    }
}
