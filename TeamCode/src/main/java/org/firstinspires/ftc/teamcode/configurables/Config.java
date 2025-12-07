package org.firstinspires.ftc.teamcode.configurables;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.GenericValue;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
public class Config {
    @Configurable
     public static class shooterConstants{
        public static double Kp = 0.0;
        public static double Kv = 0.0;
        public static double tolerance = 150;
        public static double targetRPM = 4000;
    }
    @Configurable
    public static class motorNames{
         public static String Turret    = "Turret";
        public static String FrontRight = "FrontRight";
        public static String BackRight  = "BackRight";
        public static String FrontLeft  = "FrontLeft";
        public static String BackLeft   = "BackLeft";
        public static String Camera     = "Webcam 1";
        public static String AimServo1 = "AimServo1";
        public static String AimServo2 = "AimServo2";
        public static String BlockerServo = "Blocker";
        public static String Shooter      = "Shooter";
    }
}
