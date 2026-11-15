package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

public class IntakeSubsystem extends SubsystemBase {
    private final RobotHardware robot;
    private boolean rollersRunning = false;
    private ElapsedTime jamTimer = new ElapsedTime();
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
        rollersRunning = true;
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.gateServo.setPosition(gateUpPos);
        robot.pollenLimitingServo.setPosition(pollenDownPos);
        robot.intakeMotor.setPower(intakePower);
    }

    public void runRollersNectar() {
        rollersRunning = true;
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.gateServo.setPosition(gateDownPos);
        robot.pollenLimitingServo.setPosition(pollenUpPos);
        robot.intakeMotor.setPower(intakePower);
    }

    public void runRollersFlower() {
        rollersRunning = true;
        robot.leftFlowerRampServo.setPosition(flowerRampDownPos);
        robot.rightFlowerRampServo.setPosition(flowerRampDownPos);
        robot.gateServo.setPosition(gateUpPos);
        robot.pollenLimitingServo.setPosition(pollenDownPos);
        robot.intakeMotor.setPower(intakePower);
    }

    public void increasePower() {
        intakePower = Math.min(1.0, Math.round((intakePower + 0.1) * 10.0) / 10.0);
        if (rollersRunning) {
            robot.intakeMotor.setPower(intakePower);
        }
    }

    public void decreasePower() {
        intakePower = Math.max(0.0, Math.round((intakePower - 0.1) * 10.0) / 10.0);
        if (rollersRunning) {
            robot.intakeMotor.setPower(intakePower);
        }
    }

    /**
     * Opens intake gates and runs rollers in reverse at -0.3 power to eject game pieces cleanly.
     */
    public void reverseIntake() {
        rollersRunning = false;
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.gateServo.setPosition(gateDownPos);
        robot.pollenLimitingServo.setPosition(pollenUpPos);
        robot.intakeMotor.setPower(-1);
    }

    public double getIntakeMotorPower() {
        return intakePower;
    }

    /** Set the shared intake motor power used to feed game pieces into the shooter. */
    public void setFeederPower(double power) {
        rollersRunning = false;
        robot.intakeMotor.setPower(power);
    }

    public void stop() {
        rollersRunning = false;
        robot.intakeMotor.setPower(intakeIdleSpeed);
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
    }

    /*
    public boolean isIntakeJammed () {

        if (robot.intakeMotor.getPower() <= )
        return isIntakeJammed();
    }

     */


    @Override
    public void periodic() {
        if (rollersRunning) {
            robot.intakeMotor.setPower(intakePower);
        }
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

    public class wallIntakeAction implements Action {
        @Override
        public void init() {
            runRollersPollen();
        }

        @Override
        public void loop() {}

        @Override
        public void shutdown() {
            stop();
        }

        public boolean finished() {return true;}
    }
}
