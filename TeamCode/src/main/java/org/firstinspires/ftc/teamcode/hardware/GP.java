package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;


public class GP {
    private Gamepad controller;
    public boolean x, y, a, b = false;
    public float rt,lt = 0;
    public boolean rb,lb = false;
    public boolean up, down, left, right = false;
    public float left_x, left_y, right_x, right_y = 0;
    
    public void update(Gamepad gamepad) {
        x = gamepad.x;
        y = gamepad.y;
        a = gamepad.a;
        b = gamepad.b;
        rt = gamepad.right_trigger;
        lt = gamepad.left_trigger;
        rb = gamepad.right_bumper;
        lb = gamepad.left_bumper;
        up = gamepad.dpad_up;
        down = gamepad.dpad_down;
        left = gamepad.dpad_left;
        right = gamepad.dpad_right;
        left_x = gamepad.left_stick_x;
        left_y = gamepad.left_stick_y;
        right_x = gamepad.right_stick_x;
        right_y = gamepad.right_stick_y;
    }

    public static boolean checkPressed(boolean past, boolean buttonCurrent){
        return (!past && buttonCurrent);
    }
}