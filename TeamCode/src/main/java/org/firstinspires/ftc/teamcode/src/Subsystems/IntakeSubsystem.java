package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

public class IntakeSubsystem extends SubsystemBase {
    private static final double DEFAULT_INTAKE_POWER = 0.7;
    private static final double INTAKE_RPM_AT_DEFAULT_POWER = 640.0;

    private final RobotHardware robot;
    private final double FEEDER_TICK_PER_REV = 145.1; //1150RPM gearbox 5.2:1; gear ratio 1:1 drive
    private double intakePower = DEFAULT_INTAKE_POWER;
    private boolean collectingForward;
    private double intakeMotor_CurrentTickPerSecond;
    private double intakeRPMMeasured =  Double.NaN;
    public static double intakeKp = 0.0, intakeKi = 0.0, intakeKd = 0.0, intakeKf = 1.0;

    private final PIDFController intakePIDF = new PIDFController(intakeKp, intakeKi, intakeKd, intakeKf);

    private double voltage;

    public IntakeSubsystem(RobotHardware robot) {
        this.robot = robot;
    }

    @Override
    public void periodic() {
        intakeRPMMeasured =
            robot.intakeMotor.getVelocity() * 60.0 / FEEDER_TICK_PER_REV;
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
        startForwardIntake();
    }

    public void runRollersNectar() {
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.gateServo.setPosition(gateDownPos);
        robot.pollenLimitingServo.setPosition(pollenUpPos);
        startForwardIntake();
    }

    public void runRollersFlower() {
        robot.leftFlowerRampServo.setPosition(flowerRampDownPos);
        robot.rightFlowerRampServo.setPosition(flowerRampDownPos);
        robot.gateServo.setPosition(gateUpPos);
        robot.pollenLimitingServo.setPosition(pollenDownPos);
        startForwardIntake();
    }

    private void startForwardIntake() {
        collectingForward = true;
        robot.intakeMotor.setPower(intakePower);
    }

    public void increasePower() {
        intakePower = Math.min(1.0, Math.round((intakePower + 0.1) * 10.0) / 10.0);
        if (collectingForward) {
            robot.intakeMotor.setPower(intakePower);
        }
    }

    public void decreasePower() {
        intakePower = Math.max(0.0, Math.round((intakePower - 0.1) * 10.0) / 10.0);
        if (collectingForward) {
            robot.intakeMotor.setPower(intakePower);
        }
    }

    //share intake for transfer
    public void setFeederPower(double power) {
        collectingForward = false;
        robot.intakeMotor.setPower(power);
    }
    public void setFeederRPM(double targetRPM) {
        collectingForward = false;
        // Read intake velocity once per call.
       int intakeTicksPerSecond = (int) (targetRPM * FEEDER_TICK_PER_REV / 60.0);
       robot.intakeMotor.setVelocity(intakeTicksPerSecond);
    }
    public void runFeederIntakeAtTargetRPM() {
        setFeederRPM(intakeTransferRPM);
    }

    public void stopFeederIntake() {
        setFeederRPM(0.0);
    }

public boolean feederReady(double rpmTolerance) {
    return Math.abs(getFeederCurrentRPM() - intakeTransferRPM) <= rpmTolerance;
}

    /**
     * Opens intake gates and runs rollers in reverse at -0.3 power to eject game pieces cleanly.
     */
    public void reverseIntake() {
        collectingForward = false;
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.gateServo.setPosition(gateDownPos);
        robot.pollenLimitingServo.setPosition(pollenUpPos);
        robot.intakeMotor.setPower(-1);
    }

    public void prepareEject() {
    setFeederPower(0.0);
    robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
    robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
    robot.gateServo.setPosition(gateDownPos);
    robot.pollenLimitingServo.setPosition(pollenUpPos);
    }

    public double getIntakeMotorPower() {
        return intakePower;
    }

    public double getCollectionTargetRPM() {
        return INTAKE_RPM_AT_DEFAULT_POWER * intakePower / DEFAULT_INTAKE_POWER;
    }

    public double getFeederCurrentRPM() {
        return intakeRPMMeasured;
    }

    public double getFeederTargetRPM() {
        return intakeTransferRPM;
    }

      public void getVoltage(){
        voltage = robot.voltageSensor.getVoltage();
    }

    public void stop() {
        collectingForward = false;
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
