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
    private final RobotHardware robot;

    public static double leftShooterKp = 0.05, leftShooterKi = 0.0, leftShooterKd = 0.0, leftShooterKf = 0.0002;
    public static double rightShooterKp = 0.05, rightShooterKi = 0.0, rightShooterKd = 0.0, rightShooterKf = 0.0002;

    public static double leftShooterTargetRPM = 675.0;
    public static double rightShooterTargetRPM = 675.0;

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
        robot.gateServo.setPosition(gateUpPos);

        leftShooterPIDF.setPIDF(leftShooterKp, leftShooterKi, leftShooterKd, leftShooterKf);
        rightShooterPIDF.setPIDF(rightShooterKp, rightShooterKi, rightShooterKd, rightShooterKf);

        double leftVelTicks = robot.leftShooterMotor.getVelocity();
        double leftTargetTicks = rpmToTicksPerSecond(leftShooterTargetRPM);
        double leftOutput = leftShooterPIDF.calculate(leftVelTicks, leftTargetTicks);
        robot.leftShooterMotor.setPower(Range.clip(leftOutput, 0.0, 1.0));

        double rightVelTicks = robot.rightShooterMotor.getVelocity();
        double rightTargetTicks = rpmToTicksPerSecond(rightShooterTargetRPM);
        double rightOutput = rightShooterPIDF.calculate(rightVelTicks, rightTargetTicks);
        robot.rightShooterMotor.setPower(Range.clip(rightOutput, 0.0, 1.0));
    }

    public void shootFlywheel() {
        if (!isShooting) {
            isShooting = true;
            shootingTime.reset();
            robot.pollenLimitingServo.setPosition(pollenDownPos);
        }
        robot.gateServo.setPosition(gateUpPos);

        robot.intakeMotor.setPower(0.7);

        if (shootingTime.seconds() >= 0.25) {
            robot.pollenLimitingServo.setPosition(pollenUpPos);
        } else {
            robot.pollenLimitingServo.setPosition(pollenDownPos);
        }
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
        leftShooterTargetRPM += 25.0;
        rightShooterTargetRPM += 25.0;
    }

    public void decreaseTargetRPM() {
        leftShooterTargetRPM = Math.max(0.0, leftShooterTargetRPM - 25.0);
        rightShooterTargetRPM = Math.max(0.0, rightShooterTargetRPM - 25.0);
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
    public double getShooterPower(){return robot.leftShooterMotor.getPower();}

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
            runFlywheel();
        }

        @Override
        public boolean finished() {
            return true;
        }
    }

    public class ShootAction implements Action {
        @Override
        public void loop() {
            shootFlywheel();
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
}
