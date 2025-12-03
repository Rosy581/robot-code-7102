package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GP {
     public static final Gamepad.LedEffect RedLights = new Gamepad.LedEffect.Builder()
             .addStep(1.0,0,0,1000)
             .setRepeating(true)
             .build();
     public static final Gamepad.LedEffect BlueLights = new Gamepad.LedEffect.Builder()
             .addStep(0.0,0.0,1.0,1000)
             .setRepeating(true)
             .build();
}