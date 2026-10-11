package org.firstinspires.ftc.teamcode.TeleOp.GamepadDriver.Samples;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.TeleOp.GamepadDriver.GamepadDriver;

public class SampleGamepad extends GamepadDriver {
    public static double debounce=0.1;

    Gamepad gamepad1;
    Gamepad gamepad2;

    Gamepad[] gamepads;

    public SampleGamepad(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1=gamepad1;
        this.gamepad2=gamepad2;
        gamepads= new Gamepad[]{gamepad1, gamepad2};
    }
}
