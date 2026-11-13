package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class limelightServo extends OpMode {

    private Servo limelightAdjServo;

    //apriltag 0.6-0.7
    //pollen 0.456

    public static double servoPosition = 0.5;

    @Override
    public void init() {
        limelightAdjServo = hardwareMap.get(Servo.class, "Limelight_Servo");
        limelightAdjServo.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        limelightAdjServo.setPosition(servoPosition);
    }
}
