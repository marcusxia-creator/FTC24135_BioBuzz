package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;
import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotActionConfig.*;

@TeleOp
public class Drive_N_Turn extends OpMode {

    private RobotHardware robot;

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        robot.init();
        robot.initPinpoint();
    }

    @Override
    public void loop() {
        robot.pinpoint.update();

        
    }
}
