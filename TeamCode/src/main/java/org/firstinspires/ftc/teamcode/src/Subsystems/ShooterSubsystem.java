package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

public class  ShooterSubsystem extends SubsystemBase {
    private boolean shooterOn = false;
    private RobotHardware robot;
    private double rightMotor_ticksPerSecond;
    private double rightMotor_rpmMeasured = Double.NaN;
    private double sampledVoltage = Double.NaN;
    private double shooterTargetRPM = Double.NaN;
    private boolean flywheelControlValid = true;
    private final VoltageSubsystem voltageSubsystem;
    private final PIDFController rightShooterPIDF = new PIDFController(rightShooterKp, rightShooterKi, rightShooterKd, 0);

    public ShooterSubsystem(RobotHardware robot) {
        this.robot = robot;
        this.voltageSubsystem = robot.voltageSubsystem;
    }

    @Override
    public void periodic() {
        sampleShooterSpeed();
        sampledVoltage = robot.voltageSensor.getVoltage();
        if (shooterOn) {
            updateFlywheelPower();
        }
    }

    private void sampleShooterSpeed() {
        rightMotor_ticksPerSecond = robot.rightShooterMotor.getVelocity();
        rightMotor_rpmMeasured = Math.abs(ticksPerSecondToRPM(rightMotor_ticksPerSecond));
    }

    public static double rpmToTicksPerSecond(double motorRPM) {
        return (motorRPM * shooterTicksPerRev) / 60.0;
    }

    /**
     * Converts measured encoder ticks per second to motor RPM for telemetry.
     * Motor RPM equals encoder ticks per second times sixty divided by ticks per revolution.
     */
    public static double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond * 60.0) / shooterTicksPerRev;
    }

    public void runFlywheel() {
        runFlywheel(shooterTargetVelocityRPM);
    }

    public boolean runFlywheel(double targetRPM) {
        if (!Double.isFinite(targetRPM)) {
            rightShooterPIDF.reset();
            robot.rightShooterMotor.setPower(0.0);
            shooterOn = false;
            flywheelControlValid = false;
            return false;
        }

        shooterTargetRPM = targetRPM;
        shooterOn = true;
        return flywheelControlValid;
    }

    private void updateFlywheelPower() {
        if (!Double.isFinite(shooterTargetRPM)) {
            stopFlywheelForInvalidControl();
            return;
        }

        double maxRPM = maxVelocityRPM;

        double rightTargetNorm = shooterTargetRPM / maxRPM;
        double rightCurrentNorm = rightMotor_rpmMeasured / maxRPM;

        rightTargetNorm = clamp01(rightTargetNorm);
        rightCurrentNorm = clamp01(rightCurrentNorm);

        ///updatePids();

        rightShooterPIDF.setPIDF(rightShooterKp, rightShooterKi, rightShooterKd, 0);
        double rightOutput = rightShooterPIDF.calculate(rightCurrentNorm, rightTargetNorm);
        double feedforward = rightShooterKf * rightTargetNorm;

        double power = (rightOutput + feedforward) * voltageSubsystem.getCompensationFactor();
        if (!Double.isFinite(power)) {
            stopFlywheelForInvalidControl();
            return;
        }
        robot.rightShooterMotor.setPower(Range.clip(power, 0.0, 1.0));
        flywheelControlValid = true;
    }

    private void stopFlywheelForInvalidControl() {
        rightShooterPIDF.reset();
        robot.rightShooterMotor.setPower(0.0);
        shooterOn = false;
        flywheelControlValid = false;
    }

    public void shootFlywheel() {
        openPollenGate();
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

    public void getVoltage() {
        sampledVoltage = robot.voltageSensor.getVoltage();
    }
    public double getShooterTicksPerSecond() {
        return rightMotor_ticksPerSecond;
    }

    public double getRightShooterCurrentRPM() {
        return rightMotor_rpmMeasured;
    }

    public double getTargetRPM() {
        return shooterTargetVelocityRPM;
    }

    /** Reports ready as soon as the latest measured flywheel speed reaches its target threshold. */
    public boolean isFlywheelReady() {
        double targetRPM = shooterTargetVelocityRPM;
        return  shooterOn
                && targetRPM > 0.0
                && rightMotor_rpmMeasured >= targetRPM * shooterReadyMinimumTargetFraction;
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

    public double getCurrentShooterRPM() {
        return rightMotor_rpmMeasured;
    }

    public double getShooterVelocity() {
        return rightMotor_ticksPerSecond;
    }
    public void stop() {
        shooterOn = false;
        shooterTargetRPM = Double.NaN;
        flywheelControlValid = true;
        rightShooterPIDF.reset();
        robot.rightShooterMotor.setPower(0);
        closePollenGate();
    }

    public class spoolFlywheelAction implements Action {

        @Override
        public void loop() {
            runFlywheel();
        }

        @Override
        public void shutdown() {
            stop();
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
