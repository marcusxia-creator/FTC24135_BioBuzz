package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

public class IntakeSubsystem extends SubsystemBase {
    private final RobotHardware robot;
    private boolean rollersRunning = false;
    private double sampledIntakeTicksPerSecond;
    private double appliedIntakePower = Double.NaN;

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
        rollersRunning = true;
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.pollenLimitingServo.setPosition(limiterDownPos);
        applyIntakePower(intakePower);
    }

    public void runRollersNectar() {
        rollersRunning = true;
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.pollenLimitingServo.setPosition(limiterUpPos);
        applyIntakePower(intakePower);
    }

    public void runRollersFlower() {
        rollersRunning = true;
        robot.leftFlowerRampServo.setPosition(flowerRampDownPos);
        robot.rightFlowerRampServo.setPosition(flowerRampDownPos);
        robot.pollenLimitingServo.setPosition(limiterDownPos);
        applyIntakePower(intakePower);
    }

    public void increasePower() {
        intakePower = Math.min(1.0, Math.round((intakePower + 0.1) * 10.0) / 10.0);
        if (rollersRunning) {
            applyIntakePower(intakePower);
        }
    }

    public void decreasePower() {
        intakePower = Math.max(0.0, Math.round((intakePower - 0.1) * 10.0) / 10.0);
        if (rollersRunning) {
            applyIntakePower(intakePower);
        }
    }

    /**
     * Opens intake gates and runs rollers in reverse at -1.0 power to eject game pieces cleanly.
     */
    public void reverseIntake() {
        rollersRunning = false;
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.pollenLimitingServo.setPosition(limiterUpPos);
        applyIntakePower(-1.0);
    }

    public double getIntakeMotorPower() {
        return intakePower;
    }

    public void stop() {
        rollersRunning = false;
        applyIntakePower(intakeIdleSpeed);
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
    }

    public boolean isRollersRunning() {
        return rollersRunning;
    }

    public double getIntakeRPM() {
        double motorRPM = sampledIntakeTicksPerSecond * 60.0 / intakeTicksPerRev;
        return Math.abs(motorRPM * intakeGearRatio);
    }

    /**
     * Returns true when rollers are running and intake RPM drops below threshold.
     */
    public boolean isIntakeFull() {
        if (!rollersRunning) {
            return false;
        }
        return getIntakeRPM() < intakeFullRPMThreshold;
    }

    @Override
    public void periodic() {
        sampledIntakeTicksPerSecond = robot.intakeMotor.getVelocity();
        if (rollersRunning && appliedIntakePower != intakePower) {
            applyIntakePower(intakePower);
        }
    }

    private void applyIntakePower(double power) {
        if (Double.isNaN(appliedIntakePower) || appliedIntakePower != power) {
            robot.intakeMotor.setPower(power);
            appliedIntakePower = power;
        }
    }

    public double getSampledIntakeTicksPerSecond() {
        return sampledIntakeTicksPerSecond;
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
