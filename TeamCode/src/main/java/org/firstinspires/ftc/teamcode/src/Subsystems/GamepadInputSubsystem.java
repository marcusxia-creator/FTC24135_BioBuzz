package org.firstinspires.ftc.teamcode.src.Subsystems;

import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.*;

import static java.lang.Math.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.lib.GamepadDriver.GamepadDriver;
import org.firstinspires.ftc.teamcode.lib.GamepadDriver.*;
import org.firstinspires.ftc.teamcode.lib.GamepadDriver.PrebuiltInputs.AnalogInputs.*;
import org.firstinspires.ftc.teamcode.lib.GamepadDriver.PrebuiltInputs.DigitalInputs.Hold;
import org.firstinspires.ftc.teamcode.lib.GamepadDriver.PrebuiltInputs.DigitalInputs.Switch;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;

@Config
public class GamepadInputSubsystem extends GamepadDriver {
    public static Scalar velFactor=new Scalar(6.7,metersPerSecond);
    public static Scalar angFactor=new Scalar(6.7,radiansPerSecond);
    public static Integer smoothTicks=3;
    public static double aimingThreshold=0.8;
    
    public GamepadInput<Velocity> driveInput;
    public GamepadInput<Boolean> aimLeftHive;
    public GamepadInput<Boolean> aimRightHive;

    public GamepadInput<Integer> actionSelect; //Selects what the robot is currently doing. 0 is idle, 1 is intaking, and 2 is shooting
    public GamepadInput<Integer> intakeSelect; //0 is pollen, 1 is nectar, and 2 is flower
    public GamepadInput<Boolean> shootButton;

    public GamepadInput<Boolean> increaseShooter;
    public GamepadInput<Boolean> decreaseShooter;
    public GamepadInput<Boolean> increaseIntake;
    public GamepadInput<Boolean> decreaseIntake;

    public GamepadInputSubsystem(Gamepad gamepad1, Gamepad gamepad2){
        Gamepad[] gamepads={gamepad1,gamepad2};
        driveInput=new SmoothedVel(new DualDirectVel(gamepad1,gamepad2,gamepad->(double)-gamepad.right_stick_y,gamepad->(double)gamepad.right_stick_x,gamepad->(double)gamepad.left_stick_x,
                velFactor,angFactor),smoothTicks);
        aimLeftHive=()->max(gamepad1.left_stick_y,gamepad2.left_stick_y)>aimingThreshold;
        aimRightHive=()->min(gamepad1.left_stick_y,gamepad2.left_stick_y)<-aimingThreshold;

        actionSelect=new Switch(
                new Hold(gamepads,gamepad->gamepad.b),//Idle
                new Hold(gamepads,gamepad->gamepad.dpad_right),//Intake, will differentiate later
                new Hold(gamepads,gamepad->gamepad.x)//Begin Shooting
        );
        intakeSelect=new Switch(
                new Hold(gamepads,gamepad->gamepad.dpad_right&&!(gamepad.right_bumper||gamepad.left_bumper)),//Regular Intaking
                new Hold(gamepads,gamepad->gamepad.dpad_right&&gamepad.right_bumper&&!gamepad.left_bumper),//Nectar Intaking
                new Hold(gamepads,gamepad->gamepad.dpad_right&&gamepad.left_bumper&&!gamepad.right_bumper)//Flower Intaking
        );
        shootButton=new Hold(gamepads,gamepad->gamepad.y);

        //Ion want to be writing the increase and decrease rn, but should be pretty easy
    }

}