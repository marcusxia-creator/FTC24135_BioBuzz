package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

public class IntakeSubsystem extends SubsystemBase {
    private final RobotHardware robot;
    private boolean rollersRunning = false;
    private final ElapsedTime intakeLowVelocityTimer = new ElapsedTime();
    private static final double INTAKE_FULL_DETECTION_SECONDS = 0.3;
    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    /** Set the intake servos to their safe starting positions when the OpMode starts. */
    public void setInitialServoPositions() {
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.pollenLimitingServo.setPosition(limiterDownPos);
    }

    public void runRollersPollen() {
        intakeLowVelocityTimer.reset();
        rollersRunning = true;
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.pollenLimitingServo.setPosition(limiterDownPos);
        robot.intakeMotor.setPower(intakePower);
    }

    public void runRollersNectar() {
        intakeLowVelocityTimer.reset();
        rollersRunning = true;
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.pollenLimitingServo.setPosition(limiterUpPos);
        robot.intakeMotor.setPower(intakePower);
    }

    public void runRollersFlower() {
        intakeLowVelocityTimer.reset();
        rollersRunning = true;
        robot.leftFlowerRampServo.setPosition(flowerRampDownPos);
        robot.rightFlowerRampServo.setPosition(flowerRampDownPos);
        robot.pollenLimitingServo.setPosition(limiterDownPos);
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
        robot.pollenLimitingServo.setPosition(limiterUpPos);
        robot.intakeMotor.setPower(-1);
    }

    public double getIntakeMotorPower() {
        return intakePower;
    }

    public void stop() {
        rollersRunning = false;
        robot.intakeMotor.setPower(intakeIdleSpeed);
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
    }

    /**
     * Returns true when intake encoder velocity stays below the configured threshold
     * for at least 0.3 seconds while the rollers are running.
     */
    public boolean isIntakeFull() {
        if (!rollersRunning) {
            intakeLowVelocityTimer.reset();
            return false;
        }
        if (Math.abs(robot.intakeMotor.getVelocity()) < intakeFullVelocityThreshold) {
            return intakeLowVelocityTimer.seconds() >= INTAKE_FULL_DETECTION_SECONDS;
        }
        intakeLowVelocityTimer.reset();
        return false;
    }
    ///if under 75 then bad
    /// for transfer stopping

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
