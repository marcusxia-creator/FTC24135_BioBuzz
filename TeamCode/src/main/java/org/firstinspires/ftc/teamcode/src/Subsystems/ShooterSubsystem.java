package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
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

    private ElapsedTime shootingTime = new ElapsedTime();
    private boolean isShooting = false;
    private RobotHardware robot;

    public static double leftShooterKp = 0.01, leftShooterKi = 0.0, leftShooterKd = 0.0, leftShooterKf = 0.00084;
    public static double rightShooterKp = 0.0016, rightShooterKi = 0.0, rightShooterKd = 0.00001, rightShooterKf = 0.00005;

    public static double leftShooterTargetRPM = 700.0;
    public static double rightShooterTargetRPM = 700.0;

    private double leftMotor_rpmMeasured;
    private double rightMotor_rpmMeasured;

    private final PIDFController leftShooterPIDF = new PIDFController(leftShooterKp, leftShooterKi, leftShooterKd, leftShooterKf);
    private final PIDFController rightShooterPIDF = new PIDFController(rightShooterKp, rightShooterKi, rightShooterKd, rightShooterKf);

    public ShooterSubsystem(RobotHardware robot) {
        this.robot = robot;
        robot.leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static double rpmToTicksPerSecond(double flywheelRPM) {
        double motorRPM = flywheelRPM / GEAR_RATIO;
        return (motorRPM * TICKS_PER_REV) / 60.0;
    }

    /**
     * Converts measured Encoder Ticks Per Second back to Flywheel RPM for telemetry.
     * Formula: Flywheel RPM = ( (TicksPerSec * 60) / TicksPerRev ) * Gear Ratio
     */
    public static double ticksPerSecondToRPM(double ticksPerSecond) {
        double motorRPM = (ticksPerSecond * 60.0) / TICKS_PER_REV;
        return motorRPM * GEAR_RATIO;
    }

    public void runFlywheel() {

        double left_currentTickPerSecond = robot.leftShooterMotor.getVelocity();
        double right_currentTickPerSecond = robot.leftShooterMotor.getVelocity();

        leftMotor_rpmMeasured = ticksPerSecondToRPM(left_currentTickPerSecond);
        rightMotor_rpmMeasured = ticksPerSecondToRPM(right_currentTickPerSecond);

        double voltage = robot.voltageSensor.getVoltage();
        double maxRPMDynamic = maxVelocityRPM * voltage / REF_VOLTAGE;

        double leftTargetNorm = (double) leftShooterTargetRPM / maxRPMDynamic;
        double rightTargetNorm = (double) rightShooterTargetRPM / maxRPMDynamic;
        double leftCurrentNorm = (double) leftMotor_rpmMeasured / maxRPMDynamic;
        double rightCurrentNorm = (double) rightMotor_rpmMeasured / maxRPMDynamic;

        leftTargetNorm = clamp01(leftTargetNorm);
        rightTargetNorm = clamp01(rightCurrentNorm);
        leftCurrentNorm = clamp01(leftCurrentNorm);
        rightCurrentNorm = clamp01(rightCurrentNorm);

        robot.gateServo.setPosition(gateUpPos);

        leftShooterPIDF.setPIDF(leftShooterKp, leftShooterKi, leftShooterKd, leftShooterKf);
        rightShooterPIDF.setPIDF(rightShooterKp, rightShooterKi, rightShooterKd, rightShooterKf);

        ///Depreciated
        //double leftVelTicks = robot.leftShooterMotor.getVelocity();
        //double leftTargetTicks = rpmToTicksPerSecond(leftTargetNorm);
        double leftOutput = leftShooterPIDF.calculate(leftCurrentNorm, leftTargetNorm);
        robot.leftShooterMotor.setPower(Range.clip(leftOutput, 0.0, 1.0));

        //double rightVelTicks = robot.rightShooterMotor.getVelocity();
        //double rightTargetTicks = rpmToTicksPerSecond(rightShooterTargetRPM);
        double rightOutput = rightShooterPIDF.calculate(rightCurrentNorm, rightTargetNorm);
        robot.rightShooterMotor.setPower(Range.clip(rightOutput, 0.0, 1.0));

        robot.intakeMotor.setPower(0.6);

        updatePids();
    }

    public void shootFlywheel() {
        if (!isShooting) {
            isShooting = true;
            shootingTime.reset();
            robot.pollenLimitingServo.setPosition(pollenDownPos);
        }
        robot.gateServo.setPosition(gateUpPos);

        robot.intakeMotor.setPower(0.8);

        if (shootingTime.seconds() >= 0.75) {
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

    public void setFeederPower(double power) {
        robot.intakeMotor.setPower(power);
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
    public double getShooterRPM() {
        return getLeftShooterCurrentRPM();
    }

    public double getShooterTicksPerSecond() {
        return robot.leftShooterMotor.getVelocity();
    }

    public double getLeftShooterCurrentRPM() {
        return ticksPerSecondToRPM(robot.leftShooterMotor.getVelocity());
    }

    public double getRightShooterCurrentRPM() {
        return ticksPerSecondToRPM(robot.rightShooterMotor.getVelocity());
    }

    public double getTargetRPM() {
        return leftShooterTargetRPM;
    }

    public double getTargetTicksPerSecond() {
        return rpmToTicksPerSecond(leftShooterTargetRPM);
    }

    public void stop() {
        isShooting = false;
        shootingTime.reset();

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
