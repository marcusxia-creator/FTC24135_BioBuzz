package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

public class SensorSubsystem extends SubsystemBase {
    private RobotHardware robot;
    public SensorSubsystem (RobotHardware robot) {
        this.robot = robot;
    }


}
