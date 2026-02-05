package org.firstinspires.ftc.teamcode.configurables;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class Config {
    @Configurable
    public static class shooterConstants {
        public static double tolerance = 150;
        public static double targetRPM = - 3000;
    }

    @Configurable
    public static class partNames {
        public static String Turret = "Turret";
        public static String FrontRight = "FrontRight";
        public static String BackRight = "BackRight";
        public static String FrontLeft = "FrontLeft";
        public static String BackLeft = "BackLeft";
        public static String Camera = "Webcam 1";
        public static String AimServo1 = "AimServo1";
        public static String AimServo2 = "AimServo2";
        public static String Shooter = "Shooter";
        public static String Feeder = "Feeder";
        public static String Blocker = "Blocker";
        public static String Odometry = "pinpoint";
        public static String Intake = "Intake";
    }
}