package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;

public class IntakeSubsystem extends SubsystemBase{
    private RobotHardware robot;
    private double intakePower = 0.5;
    private final double POWER_STEP = 0.1;
    private double transferPower = 1.0;

    public IntakeSubsystem (RobotHardware robot) {
        this.robot = robot;
    }
    public void runIntakeRollers (){
        robot.intakeMotor.setPower(intakePower);
    }

    public void runTransferRollers (){
        robot.intakeMotor.setPower(transferPower);
    }

    public void increasePower() {
        intakePower = Math.min(1.0, intakePower + POWER_STEP);
    }

    public void decreasePower (){
        intakePower = Math.max(0.0, intakePower - POWER_STEP);
    }
    public double getIntakeMotorPower() {
        return intakePower;
    }

    public void stop () {
        robot.intakeMotor.setPower(0);
    }
}
