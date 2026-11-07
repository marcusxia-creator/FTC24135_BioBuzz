package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

public class IntakeSubsystem extends SubsystemBase {
    public static double intakeKp = 0.001, intakeKi = 0, intakeKd = 0;
    private final RobotHardware robot;
    private double intakePower = 0.7;

    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    /** Set the intake servos to their safe starting positions when the OpMode starts. */
    public void setInitialServoPositions() {
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.gateServo.setPosition(gateUpPos);
        robot.pollenLimitingServo.setPosition(pollenDownPos);
    }

    public void runRollersPollen() {
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.gateServo.setPosition(gateUpPos);
        robot.pollenLimitingServo.setPosition(pollenDownPos);
        robot.intakeMotor.setPower(intakePower);
    }

    public void runRollersNectar() {
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.gateServo.setPosition(gateDownPos);
        robot.pollenLimitingServo.setPosition(pollenUpPos);
        robot.intakeMotor.setPower(intakePower);
    }

    public void runRollersFlower() {
        robot.leftFlowerRampServo.setPosition(flowerRampDownPos);
        robot.rightFlowerRampServo.setPosition(flowerRampDownPos);
        robot.gateServo.setPosition(gateUpPos);
        robot.pollenLimitingServo.setPosition(pollenDownPos);
        robot.intakeMotor.setPower(intakePower);
    }

    public void increasePower() {
        intakePower = Math.min(1.0, intakePower + 0.05);
        if (robot.intakeMotor.getPower() > 0) {
            robot.intakeMotor.setPower(intakePower);
        }
    }

    public void decreasePower() {
        intakePower = Math.max(0.0, intakePower - 0.05);
        if (robot.intakeMotor.getPower() > 0) {
            robot.intakeMotor.setPower(intakePower);
        }
    }

    /**
     * Opens intake gates and runs rollers in reverse at -0.3 power to eject game pieces cleanly.
     */
    public void reverseIntake() {
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.gateServo.setPosition(gateDownPos);
        robot.pollenLimitingServo.setPosition(pollenUpPos);
        robot.intakeMotor.setPower(-0.3);
    }

    public double getIntakeMotorPower() {
        return intakePower;
    }

    public void stop() {
        robot.intakeMotor.setPower(intakeIdleSpeed);
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
    }

    public class flowerIntakeAction implements Action {
        @Override
        public void init() {
            runRollersFlower();//start intake and run ramp, once when this action is called.
        }

        @Override
        public void loop() {}

        @Override
        public void shutdown() {
            stop();//Will retract ramp and intake when this action ends
        }

        @Override
        public boolean finished() {
            return true;
        }
    }
}
