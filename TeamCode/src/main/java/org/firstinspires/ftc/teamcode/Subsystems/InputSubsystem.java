package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.metersPerSecond;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.metersPerSecondSquared;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.radiansPerSecond;
import static org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Units.Unit.radiansPerSecondSquared;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.GamepadDriver.GamepadDriver;
import org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.AnalogInputs.LerpedVel;
import org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.AnalogInputs.Slider;
import org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.DigitalInputs.Button;
import org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.DigitalInputs.Switch;
import org.firstinspires.ftc.teamcode.GamepadDriver.PrebuiltInputs.DigitalInputs.Toggle;
import org.firstinspires.ftc.teamcode.IceWaddler2.src.Math.Measurement.Scalar;

public class InputSubsystem extends GamepadDriver {
    public static double debounce=0.1;

    Gamepad gamepad1;
    Gamepad gamepad2;

    Gamepad[] gamepads;

    public LerpedVel drive;
    public Switch driveSelect;
    public Switch actionSelect;

    public InputSubsystem(Gamepad gamepad1, Gamepad gamepad2){
        this.gamepad1=gamepad1;
        this.gamepad2=gamepad2;
        gamepads=new Gamepad[]{gamepad1, gamepad2};

        drive=new LerpedVel(gamepad1,gamepad2,(gamepad) -> (double)gamepad.right_stick_x,(gamepad) -> (double)-gamepad.right_stick_y,(gamepad) -> (double)gamepad.left_stick_x,new Scalar(6,metersPerSecond),new Scalar(6,radiansPerSecond),new Scalar(5,metersPerSecondSquared),new Scalar(5,radiansPerSecondSquared));

        driveSelect=new Switch(gamepads,
                gamepad->drive.active()||gamepad.b, //Manual drive, any joystick activity or cancel button b
                gamepad->gamepad.right_trigger>=0.9&&gamepad.dpad_up//semiauto
                );

        actionSelect=new Switch(gamepads,
                gamepad->gamepad.b,//Cancel, to idle
                gamepad->gamepad.a,//intake
                gamepad->gamepad.y//shoot
        );
}
}
