package org.firstinspires.ftc.teamcode.hardware;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.Gamepad;

public class InputHandler {
    private Gamepad past = new Gamepad();
    private Gamepad current = new Gamepad();
    InputHandler(){

    }
    enum INPUT{
        Y(1),
        TRIANGLE(Y.value),
        X(2),
        SQUARE(X.value),
        A(3),
        CROSS(A.value),
        B(4),
        CIRCLE(B.value),
        DPAD_UP(5),
        DPAD_DOWN(6),
        DPAD_LEFT(7),
        DPAD_RIGHT(8)
        ;
        private int value;
        INPUT(int val) {
            this.value = val;
        }
        public int value(){
            return value;
        }
    }
}
