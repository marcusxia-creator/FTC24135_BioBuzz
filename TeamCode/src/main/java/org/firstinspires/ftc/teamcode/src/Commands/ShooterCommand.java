package org.firstinspires.ftc.teamcode.src.Commands;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.DistanceShooterRPM;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig;

public class ShooterCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private final TransferSubsystem transferSubsystem;
    private final DistanceShooterRPM rpmSelector;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, TransferSubsystem transferSubsystem,
                          DistanceShooterRPM rpmSelector) {
        this.shooterSubsystem = shooterSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.transferSubsystem = transferSubsystem;
        this.rpmSelector = rpmSelector;
        addRequirements(shooterSubsystem, intakeSubsystem, transferSubsystem);
    }

    @Override
    public void initialize() {
        stopFeeding();
    }

    @Override
    public void execute() {
        double targetRPM = rpmSelector.selectRPM();
        if (!shooterSubsystem.spoolRightFlyWheel(targetRPM) ||
                !shooterSubsystem.isRightFlywheelReady(targetRPM)){
                    stopFeeding();
                    return;
                }
        intakeSubsystem.runFeederIntakeAtTargetRPM();
        transferSubsystem.setTransferSpeedRPM(RobotActionConfig.transferTargetRPM);
        transferSubsystem.runTransferAtTargetSpeed();

        if(transferSubsystem.isTransferAtTargetSpeedRPM()){
            shooterSubsystem.openPollenGate();
            }
        else {
            //shooterSubsystem.closePollenGate();
        }
    }

    @Override
    public void end(boolean interrupted) {
        stopShotting();
    }

    private void stopShotting(){
        shooterSubsystem.closePollenGate();
        intakeSubsystem.stopFeederIntake();
        transferSubsystem.stopTransfer();
        shooterSubsystem.stop();
    }
    private void stopFeeding() {
        intakeSubsystem.stopFeederIntake();
        transferSubsystem.stopTransfer();
        //shooterSubsystem.closePollenGate();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
