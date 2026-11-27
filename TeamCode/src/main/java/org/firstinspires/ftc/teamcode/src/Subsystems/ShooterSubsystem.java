package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

@Config
public class ShooterSubsystem extends SubsystemBase {
    // Gear Ratio: Motor Gear Teeth / Flywheel Gear Teeth (e.g. 30T motor gear driving 40T flywheel gear)
    public static double GEAR_RATIO = 30.0 / 40.0;

    public static double TICKS_PER_REV = 28.0;
    public static double FEEDER_TICK_PER_REV = 145.1;

    private final RobotHardware robot;
    private final VoltageSubsystem voltageSubsystem;

    public static double leftShooterKp = 0.01, leftShooterKi = 0.0, leftShooterKd = 0.0, leftShooterKf = 1.0;
    public static double rightShooterKp = 0.13, rightShooterKi = 0.000033, rightShooterKd = 0.010, rightShooterKf = 1.0;

    private double leftMotor_rpmMeasured;
    private double rightMotor_rpmMeasured = Double.NaN;
    private double rightShooterRpmPerSecond = Double.NaN;
    private long lastRightShooterSampleNanos;
    
    private double leftMotor_CurrentTickPerSecond;
    private double rightMotor_CurrentTickPerSecond;
    
    private final PIDFController leftShooterPIDF = new PIDFController(leftShooterKp, leftShooterKi, leftShooterKd, leftShooterKf);
    private final PIDFController rightShooterPIDF = new PIDFController(rightShooterKp, rightShooterKi, rightShooterKd, 0.0);
    
    private double intakeMotor_CurrentTickPerSecond;
    private double intakeRPMMeasured = 0.0;
    private double intakeMotorTargetRRM = 0.0;

    private boolean rightShooterReady;
    
    ///Constructor
    public ShooterSubsystem(RobotHardware robot) {
        this.robot = robot;
        this.voltageSubsystem = robot.voltageSubsystem;
        robot.leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    ///Depreciated
    /**
    public static double rpmToTicksPerSecond(double flywheelRPM) {
        double motorRPM = flywheelRPM / GEAR_RATIO;
        return (motorRPM * TICKS_PER_REV) / 60.0;
    }
     */

    @Override 
    public void periodic() {
        sampleRightShooterSpeed();
    }   


    public static double rpmToTicksPerSecond(double RPM, double GEAR_RATIO, double TICKS_PER_REV) {
        double motorRPM = RPM / GEAR_RATIO;
        return (motorRPM * TICKS_PER_REV) / 60.0;
    }

    /**
     * Converts measured Encoder Ticks Per Second back to Flywheel RPM for telemetry.
     * Formula: Flywheel RPM = ( (TicksPerSec * 60) / TicksPerRev ) * Gear Ratio
     */
    public static double ticksPerSecondToRPM(double ticksPerSecond, double tickPerRev, double gearRatio) {
        double motorRPM = (ticksPerSecond * 60.0) / tickPerRev;
        return motorRPM * gearRatio;
    }

    //Run 
    public void spoolRightFlyWheel() {
        spoolRightFlyWheel(rightShooterTargetRPM);
    }

    public boolean spoolRightFlyWheel(double targetRPM) {
        rightShooterTargetRPM = targetRPM;
        if (!Double.isFinite(targetRPM) || targetRPM <= 0.0
                || !Double.isFinite(maxShooterRpm) || maxShooterRpm <= 0.0
                || !Double.isFinite(rightMotor_rpmMeasured)) {
            rightShooterPIDF.reset();
            robot.rightShooterMotor.setPower(0.0);
            return false;
        }

        double normalizedTarget = targetRPM / maxShooterRpm;
        double normalizedMeasured = rightMotor_rpmMeasured / maxShooterRpm;
        rightShooterPIDF.setPIDF(rightShooterKp, rightShooterKi, rightShooterKd, 0.0);
        double pidOutput = rightShooterPIDF.calculate(normalizedMeasured, normalizedTarget);
        double feedforward = rightShooterKf * normalizedTarget;
        double power = (pidOutput + feedforward) * voltageSubsystem.getCompensationFactor();

        if (!Double.isFinite(power)) {
            rightShooterPIDF.reset();
            robot.rightShooterMotor.setPower(0.0);
            return false;
        }
        robot.rightShooterMotor.setPower(Range.clip(power, 0.0, 1.0));
        //updatePids();
        return true;
    }

    ///check if shooter is ready
    public boolean isRightFlywheelReady(double targetMotorRPM) {
        return Math.abs(rightMotor_rpmMeasured - targetMotorRPM) <= shooterToleranceRPM
                && Math.abs(rightShooterRpmPerSecond) <= rightShooterMaxRpmPerSecond;
    }

    /// for Auto Action System
    public void shootRightFlywheel() {
        if (spoolRightFlyWheel(rightShooterTargetRPM) && isRightFlywheelReady(rightShooterTargetRPM)) {
            openPollenGate();
            setFeederRPM(intakeTransferRPM);
        } else {
            setFeederRPM(0.0);
            closePollenGate();
        }
    }

    public void setRightShooterPower(double power) {
        sampleRightShooterSpeed();
        upSeparatorGate();
        robot.rightShooterMotor.setPower(Double.isFinite(power) ? Range.clip(power, 0.0, 1.0) : 0.0);
    }

    public boolean setFeederRPM(double targetRPM) {
        // Read intake velocity once per call.
        int intakeMotor_transferVeloctyTicksPerSecond = (int) rpmToTicksPerSecond(targetRPM, 1.0, FEEDER_TICK_PER_REV);
        robot.intakeMotor.setVelocity(intakeMotor_transferVeloctyTicksPerSecond);
        robot.transferSubsystem.setTransferSpeedRPM(targetRPM);
        robot.transferSubsystem.runTransferAtTargetSpeed();
        return true;
    }

    public void openPollenGate() {
        robot.pollenLimitingServo.setPosition(pollenUpPos);
    }

    public void closePollenGate() {
        robot.pollenLimitingServo.setPosition(pollenDownPos);
    }

    public void upSeparatorGate(){
        robot.gateServo.setPosition(gateUpPos);
    }

    public void downSeparatorGate(){
        robot.gateServo.setPosition(gateDownPos);
    }

    public void increaseTargetRPM() {
        //leftShooterTargetRPM += 50.0;
        rightShooterTargetRPM += 50.0;
    }

    public void decreaseTargetRPM() {
        //leftShooterTargetRPM = Math.max(0.0, leftShooterTargetRPM - 50.0);
        rightShooterTargetRPM = Math.max(0.0, rightShooterTargetRPM - 50.0);
    }

    ///Setter
    public double setShooterRPM(double rpm) {
        //leftShooterTargetRPM = rpm;
        rightShooterTargetRPM = rpm;
        return rpm;
    }
    
    /// Getter
    public void sampleRightShooterSpeed() {
        rightMotor_CurrentTickPerSecond = Math.abs(robot.rightShooterMotor.getVelocity());
        double rpm = ticksPerSecondToRPM(rightMotor_CurrentTickPerSecond, TICKS_PER_REV, 1.0);
        long now = System.nanoTime();
        double elapsedSeconds = (now - lastRightShooterSampleNanos) * 1e-9;
        rightShooterRpmPerSecond = elapsedSeconds > 0.0 && Double.isFinite(rpm)
                && Double.isFinite(rightMotor_rpmMeasured)
                ? (rpm - rightMotor_rpmMeasured) / elapsedSeconds
                : Double.NaN;
        rightMotor_rpmMeasured = rpm;
        lastRightShooterSampleNanos = now;
    }
    // Power
    public double getLeftShooterMotorPower() {
        return robot.leftShooterMotor.getPower();
    }

    public double getRightShooterMotorPower() {
        return robot.rightShooterMotor.getPower();
    }

    //Ticks - Shooter
    public double getRightShooterTicksPerSecond() {
        return rightMotor_CurrentTickPerSecond;
    }

    public double getRightShooterCurrentRPM() {
        return rightMotor_rpmMeasured;
    }

    public double getRightShooterTargetRPM() {
        return rightShooterTargetRPM;
    }

    public double getLeftShooterTicksPerSecond() {
        return leftMotor_CurrentTickPerSecond;
    }

        public double getLeftShooterCurrentRPM() {
        return leftMotor_rpmMeasured;
    }

    //Feeder

    public double getFeederMotorPower() {
        return robot.intakeMotor.getPower();
    }

    public double getFeederTicks(){
        return intakeMotor_CurrentTickPerSecond;
    }

    //RPM
    public double getFeederCurrentRPM(){
        return intakeRPMMeasured;
    }


    public double getIntakeTargetRPM() {
        return intakeMotorTargetRRM;
    }

    public void stop() {
        //leftShooterPIDF.reset();
        rightShooterPIDF.reset();
        //robot.leftShooterMotor.setPower(0);
        robot.rightShooterMotor.setPower(0);
        robot.intakeMotor.setPower(0);
        robot.transferSubsystem.stopTransfer();
        closePollenGate();
    }

    public class spoolFlywheelAction implements Action {
        @Override
        public void loop() {
            spoolRightFlyWheel();//Constantly run flywheel pid continuously without shooting
        }

        @Override
        public boolean finished() {
            return true;
        }
    }

    public class ShootAction implements Action {
        @Override
        public void loop() {
            shootRightFlywheel();//retracts limiter for shooting
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
         if(leftShooterPIDF.getP()!=leftShooterKp){
             leftShooterPIDF.setP(leftShooterKp);
         }
         if(leftShooterPIDF.getI()!=leftShooterKi){
             leftShooterPIDF.setI(leftShooterKi);
         }
         if(leftShooterPIDF.getD()!=leftShooterKd){
             leftShooterPIDF.setD(leftShooterKd);
         }
         if(leftShooterPIDF.getF()!=leftShooterKf){
             leftShooterPIDF.setF(leftShooterKf);
         }


         if(rightShooterPIDF.getP()!=rightShooterKp){
             rightShooterPIDF.setP(rightShooterKp);
         }
         if(rightShooterPIDF.getI()!=rightShooterKi){
             rightShooterPIDF.setI(rightShooterKi);
         }
         if(rightShooterPIDF.getD()!=rightShooterKd){
             rightShooterPIDF.setD(rightShooterKd);
         }
    }

}
