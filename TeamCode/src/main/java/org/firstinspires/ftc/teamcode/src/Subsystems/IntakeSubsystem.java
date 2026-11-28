package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;
///0.7 power = 640 RPM
public class IntakeSubsystem extends SubsystemBase {
    private static final double INTAKE_STARTUP_GRACE_SECONDS = 0.5;
    private final RobotHardware robot;
    private boolean rollersRunning = false;
    private boolean collectingForward;
    private double sampledIntakeTicksPerSecond;
    private double appliedIntakePower = Double.NaN;
    private final ElapsedTime intakeRunTimer = new ElapsedTime();

    private final double FEEDER_TICK_PER_REV = 145.1;

    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    public void setInitialServoPositions() {
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
    }

    public void runShooterIntake () {
        startRollers();
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        startForwardIntake(intakePower);
    }

    public void runRollersPollen() {
        startRollers();
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        startForwardIntake(intakePower);
    }

    public void runRollersNectar() {
        startRollers();
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        startForwardIntake(intakePower);
    }

    public void runRollersFlower() {
        startRollers();
        robot.leftFlowerRampServo.setPosition(flowerRampDownPos);
        robot.rightFlowerRampServo.setPosition(flowerRampDownPos);
        startForwardIntake(intakePower);
    }

    public void increasePower() {
        intakePower = Math.min(1.0, Math.round((intakePower + 0.1) * 10.0) / 10.0);
        if (rollersRunning) {
            startForwardIntake(intakePower);
        }
    }

    public void decreasePower() {
        intakePower = Math.max(0.0, Math.round((intakePower - 0.1) * 10.0) / 10.0);
        if (rollersRunning) {
            startForwardIntake(intakePower);
        }
    }

    /**
     * Opens intake gates and runs rollers in reverse at -1.0 power to eject game pieces cleanly.
     */
    public void reverseIntake() {
        rollersRunning = false;
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        startForwardIntake(-1.0);
    }

    public double getIntakeMotorPower() {
        return intakePower;
    }

    public void stop() {
        rollersRunning = false;
        startForwardIntake(intakeIdleSpeed);
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
        return intakeRunTimer.seconds() >= INTAKE_STARTUP_GRACE_SECONDS
                && getIntakeRPM() < intakeFullRPMThreshold;
    }

    private void startRollers() {
        if (!rollersRunning) intakeRunTimer.reset();
        rollersRunning = true;
    }

    @Override
    public void periodic() {
        sampledIntakeTicksPerSecond = robot.intakeMotor.getVelocity();

        /**
        if (rollersRunning && appliedIntakePower != intakePower) {
            applyIntakePower(intakePower);
        }
         */
    }

    private void setFeederRPM(double targetRPM) {
        collectingForward = false;

        int intakeTicksPerSecond = (int) (targetRPM * FEEDER_TICK_PER_REV / 60.0);
        robot.intakeMotor.setVelocity(intakeTicksPerSecond);
    }
    public void runFeederIntakeAtTargetRPM() {
        setFeederRPM(intakeTransferRPM);
    }

    private void startForwardIntake(double power) {
        collectingForward = true;
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
