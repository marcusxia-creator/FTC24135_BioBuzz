package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.src.Commands.RobotDriveCommand;
import org.firstinspires.ftc.teamcode.src.Subsystems.ControlSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.RobotDriveSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.DistanceShooterRPM;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import org.firstinspires.ftc.teamcode.src.UniversalTools.ShotTargeting;

@TeleOp(name = "BioBuzz_Marcus", group = "TeleOp")
public class BasicTeleOp extends CommandOpMode {
    private RobotDriveSubsystem robotDrive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private ControlSubsystem controls;
    private TransferSubsystem transfer;
    private GamepadEx gamepad1Ex;
    private GamepadEx gamepad2Ex;
    private ShotTargeting targeting;
    private DistanceShooterRPM shooterRPM;
    private boolean autoAimActive;

    @Override
    public void initialize() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init(false);

        robotDrive = new RobotDriveSubsystem(robot);
        intake = new IntakeSubsystem(robot);
        transfer = new TransferSubsystem(robot);
        shooter = new ShooterSubsystem(robot);
        controls = new ControlSubsystem();
        targeting = new ShotTargeting(robot.odo);
        shooterRPM = new DistanceShooterRPM(targeting);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Right stick X/Y moves and left stick X rotates; either driver can take over each axis.
        robotDrive.setDefaultCommand(new RobotDriveCommand(
                robotDrive,
                () -> Math.abs(gamepad1Ex.getRightY()) > 0.05
                        ? -gamepad1Ex.getRightY() : -gamepad2Ex.getRightY(),
                () -> Math.abs(gamepad1Ex.getRightX()) > 0.05
                        ? gamepad1Ex.getRightX() : gamepad2Ex.getRightX(),
                this::selectTurn
        ));

        controls.configureControls(gamepad1Ex, gamepad2Ex, intake, transfer, shooter, shooterRPM, targeting);
    }

    @Override
    public void preRun() {
        targeting.initializeFieldPose();
        intake.setInitialServoPositions();
        shooter.closePollenGate();
    }

    @Override
    public void run() {
        targeting.beginLoop();
        super.run();

        telemetry.addData("Intake Motor Power", intake.getIntakeMotorPower());
        telemetry.addLine("--- SHOOTER TELEMETRY ---");
        telemetry.addData("Shot mode", shooterRPM.getMode());
        telemetry.addData("RPM selection", shooterRPM.getStatus());
        telemetry.addData("Fixed manual RPM", "%.0f", DistanceShooterRPM.manualRPM);
        telemetry.addData("Alliance", targeting.getAlliance());
        telemetry.addData("Selected cell", targeting.getSelectedCell());
        telemetry.addData("Pinpoint targeting", targeting.getStatus());
        telemetry.addData("Chassis rotation", autoAimActive ? "Auto aim" : "Driver");
        if (Double.isFinite(targeting.getDistance())) {
            telemetry.addData("Target (in)", "(%.0f, %.0f)", targeting.getTargetX(), targeting.getTargetY());
            telemetry.addData("Target distance (in)", "%.1f", targeting.getDistance());
            telemetry.addData("Heading error (deg)", "%.1f", targeting.getHeadingErrorDegrees());
        }
        telemetry.addData("Left Shooter Motor Power", "%.3f", shooter.getLeftShooterMotorPower());
        telemetry.addData("Right Shooter Motor Power", "%.3f", shooter.getRightShooterMotorPower());
        telemetry.addData("Right Shooter Target RPM", "%.0f", shooter.getRightShooterTargetRPM());
        telemetry.addData("Right Shooter Actual Speed tps", "%.1f",shooter.getRightShooterTicksPerSecond());
        telemetry.addData("Right Shooter Actual Speed RPM", "%.1f",shooter.getRightShooterCurrentRPM());
        telemetry.addData("shooter is at RPM or not", shooter.isRightFlywheelReady(shooter.getRightShooterTargetRPM()));
        telemetry.addData("Feeder Configured RPM", "%.0f", intake.getFeederTargetRPM());
        telemetry.addData("Intake Actual RPM", "%.0f", intake.getFeederCurrentRPM());
        telemetry.addData("Transfer Target RPM", "%.0f", transfer.getFeederTargetRPM());
        telemetry.addData("Transfer Actual RPM", "%.0f", transfer.getTransferSpeedRPM());
        telemetry.update();
    }

    private double selectTurn() {
        double autoTurn = gamepad1.left_trigger > 0.5 ? targeting.getAimTurn() : Double.NaN;
        boolean aiming = Double.isFinite(autoTurn);
        if (aiming != autoAimActive || (aiming && autoTurn == 0.0)) {
            robotDrive.clearTurnSmoothing();
        }
        autoAimActive = aiming;
        if (aiming) {
            return autoTurn;
        }
        double primaryTurn = gamepad1Ex.getLeftX();
        return Math.abs(primaryTurn) > 0.05 ? primaryTurn : gamepad2Ex.getLeftX();
    }
}
