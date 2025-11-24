package org.firstinspires.ftc.teamcode.configurables;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.GenericValue;
@Configurable
public class Config {
    enum teamColor{
        RED,
        BLUE
    }
     public static class shooterConstants{
        public static double K = 0.0;
        public static double Kp = 0.0;
        public static double Kv = 0.0;
        public static double tolerance = 150;
        public static double targetRPM = 4000;
    }
}
