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
    public static double FEEDER_TICK_PER_REV = 384.5;

    private RobotHardware robot;

    public static double leftShooterKp = 0.01, leftShooterKi = 0.0, leftShooterKd = 0.0, leftShooterKf = 1.0;
    public static double rightShooterKp = 0.13, rightShooterKi = 0.000033, rightShooterKd = 0.010, rightShooterKf = 0.96;

    public static double leftShooterTargetRPM = 700.0;
    public static double rightShooterTargetRPM = 3430.0;
    private double rightShooter_maxRPMDynamic;

    public static double intakeKp = 0.0, intakeKi = 0.0, intakeKd = 0.0, intakeKf = 1.0;

    private static double intakeMotorTargetRRM = 640;

    private double leftMotor_rpmMeasured;
    private double rightMotor_rpmMeasured;
    private double intakeRPMMeasured;

    private double leftMotor_CurrentTickPerSecond;
    private double rightMotor_CurrentTickPerSecond;
    private double intakeMotor_CurrentTickPerSecond;

    private double voltage;

    private final PIDFController leftShooterPIDF = new PIDFController(leftShooterKp, leftShooterKi, leftShooterKd, leftShooterKf);
    private final PIDFController rightShooterPIDF = new PIDFController(rightShooterKp, rightShooterKi, rightShooterKd, rightShooterKf);
    private final PIDFController intakePIDF = new PIDFController(intakeKp, intakeKi, intakeKd, intakeKf);

    private boolean rightShooterReady;
    ///Constructor
    public ShooterSubsystem(RobotHardware robot) {
        this.robot = robot;
        robot.leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    ///Depreciated
    /**
    public static double rpmToTicksPerSecond(double flywheelRPM) {
        double motorRPM = flywheelRPM / GEAR_RATIO;
        return (motorRPM * TICKS_PER_REV) / 60.0;
    }
     */

    public static double rpmToTicksPerSecond(double RPM, double GEAR_RATIO, double TICKS_PER_REV) {
        double motorRPM = RPM / GEAR_RATIO;
        return (motorRPM * TICKS_PER_REV) / 60.0;
    }

    /**
     * Converts measured Encoder Ticks Per Second back to Flywheel RPM for telemetry.
     * Formula: Flywheel RPM = ( (TicksPerSec * 60) / TicksPerRev ) * Gear Ratio
     */
    /** Depreciated
    public static double ticksPerSecondToRPM(double ticksPerSecond) {
        double motorRPM = (ticksPerSecond * 60.0) / TICKS_PER_REV;
        return motorRPM * GEAR_RATIO;
    }
     */
    public static double ticksPerSecondToRPM(double ticksPerSecond, double tickPerRev, double gearRatio) {
        double motorRPM = (ticksPerSecond * 60.0) / tickPerRev;
        return motorRPM * gearRatio;
    }

    public void spoolRightFlyWheel() {
        getVoltage();
        rightMotor_CurrentTickPerSecond = robot.rightShooterMotor.getVelocity();

        rightMotor_rpmMeasured = ticksPerSecondToRPM(rightMotor_CurrentTickPerSecond,TICKS_PER_REV,GEAR_RATIO);

        rightShooter_maxRPMDynamic = maxShooterRpm * voltage / REF_VOLTAGE;

        double rightTargetNorm = (double) rightShooterTargetRPM / rightShooter_maxRPMDynamic;
         double rightCurrentNorm = (double) rightMotor_rpmMeasured / rightShooter_maxRPMDynamic;

        rightTargetNorm = clamp01(rightTargetNorm);
        rightCurrentNorm = clamp01(rightCurrentNorm);

        robot.gateServo.setPosition(gateUpPos);

        rightShooterPIDF.setPIDF(rightShooterKp, rightShooterKi, rightShooterKd, rightShooterKf);

        double rightOutput = rightShooterPIDF.calculate(rightCurrentNorm, rightTargetNorm);
        robot.rightShooterMotor.setPower(Range.clip(rightOutput, 0.0, 1.0));
        updatePids();
    }

    public void spoolLeftFlyWheel() {
        getVoltage();
        leftMotor_CurrentTickPerSecond = robot.leftShooterMotor.getVelocity();

        leftMotor_rpmMeasured = ticksPerSecondToRPM(leftMotor_CurrentTickPerSecond,TICKS_PER_REV,GEAR_RATIO);

        double maxRPMDynamic = maxShooterRpm * voltage / REF_VOLTAGE;

        double leftTargetNorm = (double) leftShooterTargetRPM / maxRPMDynamic;
        double leftCurrentNorm = (double) leftMotor_rpmMeasured / maxRPMDynamic;

        leftTargetNorm = clamp01(leftTargetNorm);
        leftCurrentNorm = clamp01(leftCurrentNorm);

        leftShooterPIDF.setPIDF(leftShooterKp, leftShooterKi, leftShooterKd, leftShooterKf);

        ///Depreciated
        double leftOutput = leftShooterPIDF.calculate(leftCurrentNorm, leftTargetNorm);
        robot.leftShooterMotor.setPower(Range.clip(leftOutput, 0.0, 1.0));
        updatePids();
        upSeparatorGate();
        openPollenGate();
    }

    public void shootRightFlywheel() {
        spoolRightFlyWheel();
        updateFeederWhenReady(intakeMotorTargetRRM);
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

    public void updateFeederWhenReady(double feederRPM) {
        // Small tolerance prevents switching from tiny RPM fluctuations.
        double rpmTolerance = 50.0;

        boolean targetsActive =
                leftShooterTargetRPM > 0 && rightShooterTargetRPM > 0;

        rightShooterReady =
               rightMotor_rpmMeasured >= Math.max(0, rightShooterTargetRPM - rpmTolerance);

        if (targetsActive && rightShooterReady) {
            setFeederRPM(feederRPM);
        } else {
            setFeederRPM(0.0);
        }
    }

    public void setFeederPower(double power) {
        robot.intakeMotor.setPower(power);
    }
    public void setFeederRPM(double targetRPM) {
        // Read intake velocity once per call.
        intakeMotor_CurrentTickPerSecond = robot.intakeMotor.getVelocity();
        intakeRPMMeasured = intakeMotor_CurrentTickPerSecond * 60.0 / FEEDER_TICK_PER_REV;

        double maxRPMDynamic = maxFeederRpm * voltage / REF_VOLTAGE;

        if (targetRPM <= 0.0) {
            intakePIDF.reset();
            robot.intakeMotor.setPower(0.0);
            return;
        }

        intakePIDF.setPIDF(intakeKp, intakeKi, intakeKd, intakeKf);

        double targetNorm = clamp01(targetRPM / maxRPMDynamic);
        double currentNorm = intakeRPMMeasured / maxRPMDynamic;
        double output = intakePIDF.calculate(currentNorm, targetNorm);
        robot.intakeMotor.setPower(Range.clip(output, 0.0, 1.0));
    }

    public boolean feederReady(double rpmTolerance){
        return intakeRPMMeasured >= Math.max(0, intakeMotorTargetRRM - rpmTolerance);
    }


    public void increaseTargetRPM() {
        leftShooterTargetRPM += 50.0;
        rightShooterTargetRPM += 50.0;
    }

    public void decreaseTargetRPM() {
        leftShooterTargetRPM = Math.max(0.0, leftShooterTargetRPM - 50.0);
        rightShooterTargetRPM = Math.max(0.0, rightShooterTargetRPM - 50.0);
    }

    ///Setter
    public double setShooterRPM(double rpm) {
        leftShooterTargetRPM = rpm;
        rightShooterTargetRPM = rpm;
        return rpm;
    }
    ///
    /// Getter
    public void getVoltage(){
        voltage = robot.voltageSensor.getVoltage();
    }
    // Power
    public double getLeftShooterMotorPower() {
        return robot.leftShooterMotor.getPower();
    }

    public double getRightShooterMotorPower() {
        return robot.rightShooterMotor.getPower();
    }

    public double getFeederMotorPower() {
        return robot.intakeMotor.getPower();
    }
    //Ticks
    public double getRightShooterTicksPerSecond() {
        return rightMotor_CurrentTickPerSecond;
    }

    public double getLeftShooterTicksPerSecond() {
        return leftMotor_CurrentTickPerSecond;
    }
    public double getFeederTicks(){
        return intakeMotor_CurrentTickPerSecond;
    }

    //RPM
    public double getFeederCurrentRPM(){
        return intakeRPMMeasured;
    }

    public double getLeftShooterCurrentRPM() {
        return leftMotor_rpmMeasured;
    }

    public double getRightShooterCurrentRPM() {
        return rightMotor_rpmMeasured;
    }

    public double getRightShooterTargetRPM() {
        return rightShooterTargetRPM;
    }

    public double getIntakeTargetRPM() {
        return intakeMotorTargetRRM;
    }

    public double getRightShooter_maxRPMDynamic() {
        return rightShooter_maxRPMDynamic;
    }

    public double getTargetTicksPerSecond() {
        return rpmToTicksPerSecond(leftShooterTargetRPM,TICKS_PER_REV, GEAR_RATIO);
    }

    public void stop() {
        leftShooterPIDF.reset();
        rightShooterPIDF.reset();
        robot.leftShooterMotor.setPower(0);
        robot.rightShooterMotor.setPower(0);
        robot.intakeMotor.setPower(0);
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
         if(rightShooterPIDF.getF()!=rightShooterKf){
             rightShooterPIDF.setF(rightShooterKf);
         }
    }

    private static double clamp01(double x) {
        return x < 0 ? 0 : (x > 1 ? 1 : x);
    }

}
