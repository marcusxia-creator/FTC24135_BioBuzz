package org.firstinspires.ftc.teamcode.TeleOp.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotHardware;
import static org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.RobotActionConfig.*;

public class IntakeSubsystem extends SubsystemBase {
    public static double intakeKp = 0.001, intakeKi = 0, intakeKd = 0;
    private final RobotHardware robot;
    private double intakePower = 0.7;

    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    public void runRollersPollen() {
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.gateServo.setPosition(gateDownPos);
        robot.pollenLimitingServo.setPosition(pollenDownPos);
        robot.intakeMotor.setPower(intakePower);
    }

    public void runRollersNectar() {
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.gateServo.setPosition(gateUpPos);
        robot.pollenLimitingServo.setPosition(pollenUpPos);
        robot.intakeMotor.setPower(intakePower);
    }

    public void runRollersFlower() {
        robot.leftFlowerRampServo.setPosition(flowerRampDownPos);
        robot.rightFlowerRampServo.setPosition(flowerRampDownPos);
        robot.gateServo.setPosition(gateDownPos);
        robot.pollenLimitingServo.setPosition(pollenDownPos);
        robot.intakeMotor.setPower(intakePower);
    }

    public void increasePower() {
        intakePower = Math.min(1.0, intakePower + 0.1);
        if (robot.intakeMotor != null && robot.intakeMotor.getPower() > 0) {
            robot.intakeMotor.setPower(intakePower);
        }
    }

    public void decreasePower() {
        intakePower = Math.max(0.0, intakePower - 0.1);
        if (robot.intakeMotor != null && robot.intakeMotor.getPower() > 0) {
            robot.intakeMotor.setPower(intakePower);
        }
    }

    public double getIntakeMotorPower() {
        return intakePower;
    }

    public void stop() {
        robot.intakeMotor.setPower(0);
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
    }
}
