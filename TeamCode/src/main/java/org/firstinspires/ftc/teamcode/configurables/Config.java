package org.firstinspires.ftc.teamcode.configurables;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.hardware.Robot.TEAMCOLOR;

@Configurable
public class Config {
    @Configurable
    public static class autoColor{
        public static TEAMCOLOR teamcolor = TEAMCOLOR.BLUE;
    }
    @Configurable
    public static class shooterConstants {
        public static double tolerance = 150;
        public static double targetRPMfar = - 4500;
        public static double targetRPMclose = - 3000;
    }

    @Configurable
    public static class partNames {
        public static String Turret = "Turret";
        public static String FrontRight = "FrontRight";
        public static String BackRight = "BackRight";
        public static String FrontLeft = "FrontLeft";
        public static String BackLeft = "BackLeft";
        public static String Camera = "Webcam 1";
        public static String Shooter = "Shooter";
        public static String Odometry = "pinpoint";
        public static String Intake = "Intake";
        public static String Feeder = "Feeder ";
    }
}