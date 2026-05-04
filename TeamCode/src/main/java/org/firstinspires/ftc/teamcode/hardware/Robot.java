package org.firstinspires.ftc.teamcode.hardware;
import androidx.annotation.NonNull;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot extends RobotBase {
    private GoBildaPinpointDriver odo;
    public Robot(@NonNull HardwareMap _hardwareMap, OPMODETYPE _opModeType) {
        super(_hardwareMap, _opModeType);
        switch (opModeType){
            case TELE:
                configOdo();
                break;
            case AUTO:
            case TEST:
                break;
            default:
                throw new IllegalArgumentException("NO OPMODE TYPE SPECIFIED");
        }

    }
    public void update(TEAMCOLOR teamcolor){
        odo.update();
    }
}
