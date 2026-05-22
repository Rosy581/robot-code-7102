package org.firstinspires.ftc.teamcode.hardware;

import com.bylazar.configurables.annotations.Configurable;
import org.firstinspires.ftc.teamcode.hardware.Robot.DIRECTION;


public class Config {
    @Configurable
    public static class tuningVariables{
    }

    @Configurable
    public static class partNames {
        public static String FrontRight = "FrontRight";
        public static String BackRight = "BackRight";
        public static String FrontLeft = "FrontLeft";
        public static String BackLeft = "BackLeft";
        public static String Odometry = "pinpoint";
        public static String Intake = "Intake";
    }
    @Configurable
    public static class partDirection{
        public static DIRECTION FrontRight = DIRECTION.BACKWARD;
        public static DIRECTION BackRight  = DIRECTION.BACKWARD;
        public static DIRECTION FrontLeft  = DIRECTION.FORWARD;
        public static DIRECTION BackLeft   = DIRECTION.FORWARD;
        public static DIRECTION Odometry   = DIRECTION.FORWARD;
        public static DIRECTION Intake     = DIRECTION.FORWARD;
        public static DIRECTION odoX       = DIRECTION.BACKWARD;
        public static DIRECTION odoY       = DIRECTION.FORWARD;
    }
}