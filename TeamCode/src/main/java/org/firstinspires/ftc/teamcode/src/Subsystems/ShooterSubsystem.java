package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

public class  ShooterSubsystem extends SubsystemBase {
    private ElapsedTime shootingTime = new ElapsedTime();
    private boolean isShooting = false;
    private boolean shooterOn = false;
    private RobotHardware robot;
    private final TransferSubsystem transferSubsystem;

    private double rightMotor_rpmMeasured;

    private final ElapsedTime flywheelReadyClock = new ElapsedTime();

    private final PIDFController rightShooterPIDF = new PIDFController(rightShooterKp, rightShooterKi, rightShooterKd, rightShooterKf);

    public ShooterSubsystem(RobotHardware robot, TransferSubsystem transferSubsystem) {
        this.robot = robot;
        this.transferSubsystem = transferSubsystem;
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

        rightMotor_rpmMeasured = Math.abs(ticksPerSecondToRPM(right_currentTickPerSecond));

        double voltage = robot.voltageSensor.getVoltage();
        double maxRPMDynamic = maxVelocityRPM * voltage / REF_VOLTAGE;

        double rightTargetNorm = shooterTargetVelocityRPM / maxRPMDynamic;
        double rightCurrentNorm = (double) rightMotor_rpmMeasured / maxRPMDynamic;

        rightTargetNorm = clamp01(rightTargetNorm);
        rightCurrentNorm = clamp01(rightCurrentNorm);

        rightShooterPIDF.setPIDF(rightShooterKp, rightShooterKi, rightShooterKd, rightShooterKf);

        double rightOutput = rightShooterPIDF.calculate(rightCurrentNorm, rightTargetNorm);
        robot.rightShooterMotor.setPower(Range.clip(rightOutput, 0.0, 1.0));

        updatePids();
    }

    public void shootFlywheel() {
        if (!isShooting) {
            isShooting = true;
            shootingTime.reset();
            robot.pollenLimitingServo.setPosition(limiterDownPos);
        }
        transferSubsystem.runTransfer();

        if (shootingTime.seconds() >= 0) {
            robot.pollenLimitingServo.setPosition(limiterUpPos);
        } else {
            robot.pollenLimitingServo.setPosition(limiterDownPos);
        }

        updatePids();
    }

    public void openPollenGate() {
        robot.pollenLimitingServo.setPosition(limiterUpPos);
    }

    public void closePollenGate() {
        robot.pollenLimitingServo.setPosition(limiterDownPos);
    }

    public void increaseTargetRPM() {
        shooterTargetVelocityRPM += 50.0;
    }

    public void decreaseTargetRPM() {
        shooterTargetVelocityRPM = Math.max(0.0, shooterTargetVelocityRPM - 50.0);
    }

    ///Setter
    public double setShooterRPM(double rpm) {
        shooterTargetVelocityRPM = rpm;
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
        return Math.abs(ticksPerSecondToRPM(getShooterTicksPerSecond()));
    }

    public double getTargetRPM() {
        return shooterTargetVelocityRPM;
    }

    /**
     * Reports ready after the flywheel stays above the minimum target speed
     * for the configured stability window.
     */
    public boolean isFlywheelReady() {
        double currentRPM = getRightShooterCurrentRPM();
        double minimumReadyRPM = shooterTargetVelocityRPM * shooterReadyMinimumTargetFraction;

        if (currentRPM < minimumReadyRPM) {
            flywheelReadyClock.reset();
            return false;
        }

        return flywheelReadyClock.seconds() >= shooterReadyStabilityWindowSeconds;
    }

    public void resetFlywheelReadiness() {
        flywheelReadyClock.reset();
    }

    public int getShooterEncoderPosition() {
        return robot.rightShooterMotor.getCurrentPosition();
    }

    public boolean isShooterOn() {
        return shooterOn;
    }
    public void runUsingPower (){
        robot.rightShooterMotor.setPower(0.1);
    }

    public double getTargetTicksPerSecond() {
        return rpmToTicksPerSecond(shooterTargetVelocityRPM);
    }
    public void stop() {
        shooterOn = false;
        isShooting = false;
        shootingTime.reset();
        rightShooterPIDF.reset();
        robot.rightShooterMotor.setPower(0);
        transferSubsystem.stopTransfer();
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
