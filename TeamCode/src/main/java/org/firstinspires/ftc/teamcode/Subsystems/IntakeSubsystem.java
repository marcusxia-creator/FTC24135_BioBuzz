package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.CommandBase.Action;

public class IntakeSubsystem {
    DcMotorEx intakeMotor;
    public IntakeSubsystem(RobotHardware robot){
        intakeMotor=robot.intakeMotor;
    }

    public class IntakeAction implements Action {
        @Override
        public void init() {
            intakeMotor.setPower(-7);
        }

        @Override
        public void loop() {}

        @Override
        public void shutdown() {
            intakeMotor.setPower(0);
        }
    }
}
