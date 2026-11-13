package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

public class ShooterSubsystem extends SubsystemBase {
    private ElapsedTime shootingTime = new ElapsedTime();
    private boolean isShooting = false;
    private boolean shooterOn = false;
    private RobotHardware robot;

    private double rightMotor_rpmMeasured;
    private double previousShooterRPM = -1.0;
    private double shooterReferenceRPM = 0.0;
    private int stableShooterSamples = 0;
    private boolean shooterReferenceCaptured = false;

    private final PIDFController rightShooterPIDF = new PIDFController(rightShooterKp, rightShooterKi, rightShooterKd, rightShooterKf);

    public ShooterSubsystem(RobotHardware robot) {
        this.robot = robot;
        robot.rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static double rpmToTicksPerSecond(double flywheelRPM) {
        double motorRPM = flywheelRPM / shooterGearRatio;
        return (motorRPM * shooterTicksPerRev) / 60.0;
    }

    /**
     * Converts measured Encoder Ticks Per Second back to Flywheel RPM for telemetry.
     * Flywheel RPM equals encoder ticks per second times sixty divided by ticks per revolution, then multiplied by gear ratio.
     */
    public static double ticksPerSecondToRPM(double ticksPerSecond) {
        double motorRPM = (ticksPerSecond * 60.0) / shooterTicksPerRev;
        return motorRPM * shooterGearRatio;
    }

    public void runFlywheel() {
        shooterOn = true;

        double right_currentTickPerSecond = robot.rightShooterMotor.getVelocity();

        rightMotor_rpmMeasured = ticksPerSecondToRPM(right_currentTickPerSecond);

        double voltage = robot.voltageSensor.getVoltage();
        double maxRPMDynamic = maxVelocityRPM * voltage / REF_VOLTAGE;

        double rightTargetNorm = rightShooterTargetRPM / maxRPMDynamic;
        double rightCurrentNorm = (double) rightMotor_rpmMeasured / maxRPMDynamic;

        rightTargetNorm = clamp01(rightTargetNorm);
        rightCurrentNorm = clamp01(rightCurrentNorm);

        robot.gateServo.setPosition(gateUpPos);

        rightShooterPIDF.setPIDF(rightShooterKp, rightShooterKi, rightShooterKd, rightShooterKf);

        double rightOutput = rightShooterPIDF.calculate(rightCurrentNorm, rightTargetNorm);
        robot.rightShooterMotor.setPower(Range.clip(rightOutput, 0.0, 1.0));

        updatePids();
    }

    public void shootFlywheel() {
        if (!isShooting) {
            isShooting = true;
            shootingTime.reset();
            robot.pollenLimitingServo.setPosition(pollenDownPos);
        }
        robot.gateServo.setPosition(gateUpPos);

        robot.intakeMotor.setPower(intakeShootingPower);

        if (shootingTime.seconds() >= 0) {
            robot.pollenLimitingServo.setPosition(pollenUpPos);
        } else {
            robot.pollenLimitingServo.setPosition(pollenDownPos);
        }

        updatePids();
    }

    public void openPollenGate() {
        robot.pollenLimitingServo.setPosition(pollenUpPos);
    }

    public void closePollenGate() {
        robot.pollenLimitingServo.setPosition(pollenDownPos);
    }

    public void increaseTargetRPM() {
        rightShooterTargetRPM += 50.0;
    }

    public void decreaseTargetRPM() {
        rightShooterTargetRPM = Math.max(0.0, rightShooterTargetRPM - 50.0);
    }

    ///Setter
    public double setShooterRPM(double rpm) {
        rightShooterTargetRPM = rpm;
        return rpm;
    }
    ///
    /// Getter
    public double getShooterRPM() {
        return getRightShooterCurrentRPM();
    }

    public double getShooterTicksPerSecond() {
        return robot.rightShooterMotor.getVelocity();
    }

    public double getRightShooterCurrentRPM() {
        return ticksPerSecondToRPM(robot.rightShooterMotor.getVelocity());
    }

    public double getTargetRPM() {
        return rightShooterTargetRPM;
    }

    public boolean isShooterOn() {
        return shooterOn;
    }

    public void resetShooterSpeedReference() {
        previousShooterRPM = -1.0;
        shooterReferenceRPM = 0.0;
        stableShooterSamples = 0;
        shooterReferenceCaptured = false;
    }

    /** Learn the flywheel speed when it has stabilized before feeding begins. */
    public boolean updateShooterSpeedReference() {
        double currentRPM = getShooterRPM();

        if (!shooterReferenceCaptured) {
            if (previousShooterRPM >= 0.0
                    && Math.abs(currentRPM - previousShooterRPM) <= shooterReferenceStableDeltaRPM) {
                stableShooterSamples++;
            } else {
                stableShooterSamples = 0;
            }

            if (stableShooterSamples >= shooterReferenceStableSamples) {
                shooterReferenceRPM = currentRPM;
                shooterReferenceCaptured = true;
            }

            previousShooterRPM = currentRPM;
        }

        return shooterReferenceCaptured && Math.abs(currentRPM - shooterReferenceRPM) <= shooterVelocityToleranceRPM;
    }

    public double getShooterReferenceRPM() {
        return shooterReferenceRPM;
    }

    public boolean isShooterReferenceCaptured() {
        return shooterReferenceCaptured;
    }

    // Original target-based readiness check
    // public boolean isShooterAtVelocity() {
    //     double toleranceTicksPerSecond = rpmToTicksPerSecond(shooterVelocityToleranceRPM);
    //     double targetTicksPerSecond = rpmToTicksPerSecond(rightShooterTargetRPM);
    //     double actualTicksPerSecond = robot.rightShooterMotor.getVelocity();
    //     return Math.abs(actualTicksPerSecond - targetTicksPerSecond) <= toleranceTicksPerSecond;
    // }

    public double getTargetTicksPerSecond() {
        return rpmToTicksPerSecond(rightShooterTargetRPM);
    }
    public void stop() {
        shooterOn = false;
        isShooting = false;
        shootingTime.reset();
        resetShooterSpeedReference();

        rightShooterPIDF.reset();
        robot.rightShooterMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        closePollenGate();
    }

    public class spoolFlywheelAction implements Action {
        @Override
        public void loop() {
            runFlywheel();//Constantly run flywheel pid continuously without shooting
        }

        @Override
        public boolean finished() {
            return true;
        }
    }

    public class ShootAction implements Action {
        @Override
        public void loop() {
            shootFlywheel();//retracts limiter for shooting
        }

        @Override
        public void shutdown() {
            stop();
        }

        @Override
        public boolean finished() {
            return true;
        }
    }

    //Check pid controllers against coeff variables, and updates if needed.
    void updatePids(){
        if(rightShooterPIDF.getP()!=rightShooterKp){
            rightShooterPIDF.setP(rightShooterKp);
        }
        if(rightShooterPIDF.getI()!=rightShooterKi){
            rightShooterPIDF.setI(rightShooterKi);
        }
        if(rightShooterPIDF.getD()!=rightShooterKd){
            rightShooterPIDF.setD(rightShooterKd);
        }
        if(rightShooterPIDF.getF()!=rightShooterKf){
            rightShooterPIDF.setF(rightShooterKf);
        }
    }

    private static double clamp01(double x) {
        if (x < 0) {
            return 0;
        }
        if (x > 1) {
            return 1;
        }
        return x;
    }

}
