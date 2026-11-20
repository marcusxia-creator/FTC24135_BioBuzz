package org.firstinspires.ftc.teamcode.src.Subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.DistanceShooterRPM;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.*;

public class TransferSubsystem extends SubsystemBase {
    private static final double TRANSFER_RPM_TOLERANCE = transferToleranceRPM;
    // Gear Ratio: Motor Gear Teeth / Flywheel Gear Teeth (e.g. 30T motor gear driving 40T flywheel gear)
    public static double GEAR_RATIO = 20.0 / 20.0;

    public static double FEEDER_TICK_PER_REV = 145.1; // 1150RPM gearbox 5.2:1; gear ratio 1:1 drive

    private RobotHardware robot;

    private double transferSpeedTargetRPM = transferTargetRPM;
    private double transferSpeedRPM = Double.NaN;
    private double currentVelocityTicksPerSeconds;

    public TransferSubsystem(RobotHardware robot) {
        this.robot = robot;
    }
    @Override 
    public void periodic() {

        sampleTransferVelocity();
        //setTransferSpeedRPM(transferTargetRPM);
    }

    ///conversion 
    private static double ticksPerSecondToRPM(double ticksPerSecond, double tickPerRev, double gearRatio) {
        double motorRPM = (ticksPerSecond * 60.0) / tickPerRev;
        return motorRPM * gearRatio;
    }

    public static double rpmToTicksPerSecond(double RPM, double GEAR_RATIO, double TICKS_PER_REV) {
        double motorRPM = RPM / GEAR_RATIO;
        return (motorRPM * TICKS_PER_REV) / 60.0;
    }

    public void sampleTransferVelocity(){
        transferSpeedRPM = ticksPerSecondToRPM(robot.transferMotor.getVelocity(), FEEDER_TICK_PER_REV, GEAR_RATIO);
    }
    //setter
    public void setTransferSpeedRPM(double rpm) {
        transferSpeedTargetRPM = rpm;
    }

    public void runTransferAtTargetSpeed() {  
        double targetVelocityTicksPerSeconds  = rpmToTicksPerSecond(transferSpeedTargetRPM, GEAR_RATIO, FEEDER_TICK_PER_REV);
        robot.transferMotor.setVelocity(targetVelocityTicksPerSeconds);
    }
    public boolean stopTransfer() {
        robot.transferMotor.setPower(0.0);
        return true;
    }
    
    ///Getter
    public double getTransferSpeedRPM() {
       return transferSpeedRPM;
    }

    public double getFeederTargetRPM() {
        return transferSpeedTargetRPM;
    }


    public boolean isTransferAtTargetSpeedRPM() {
        double currentRPM = getTransferSpeedRPM();
        return transferSpeedTargetRPM > 0.0
                && Double.isFinite(currentRPM)
                && currentRPM > 0.0
                && Math.abs(currentRPM - transferSpeedTargetRPM) <= TRANSFER_RPM_TOLERANCE;
    }


    public void increaseRPM() {
        transferTargetRPM = Math.min(1150,transferSpeedTargetRPM + 25.0);
        //setTransferSpeedRPM(transferSpeedTargetRPM);
        //runTransferAtTargetSpeed();
    }

    public void decreaseRPM() {
        transferTargetRPM = Math.max(0.0, transferSpeedTargetRPM - 25.0);
        //setTransferSpeedRPM(transferSpeedTargetRPM);
        //runTransferAtTargetSpeed();
    }
}
