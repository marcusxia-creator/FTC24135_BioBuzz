package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.src.Commands.RobotDriveCommand;
import org.firstinspires.ftc.teamcode.src.Subsystems.ControlSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.RobotDriveSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.TransferSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.DistanceShooterRPM;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import org.firstinspires.ftc.teamcode.src.UniversalTools.ShotTargeting;
import org.firstinspires.ftc.teamcode.TeleOp.UniversalTools.JoystickDriveProcessor;


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
    private ShotTargeting.Cell displayedCell;
    private LimelightSubsystem limelight;
    private DistanceShooterRPM shooterRPM;
    private boolean autoAimActive;

    private JoystickDriveProcessor.Output drive;
    private JoystickDriveProcessor.Output operator;

    private final ElapsedTime driveTimer = new ElapsedTime();
    // Starter settings: deadzone, exponent, acceleration, deceleration.
    // Translation: 0 -> full power in 0.5 s; full -> zero in 0.25 s.
    private final JoystickDriveProcessor joystick = new JoystickDriveProcessor(
            new JoystickDriveProcessor.JoystickResponseConfig(0.05, 2.0, 2.0, 4.0),
            new JoystickDriveProcessor.JoystickResponseConfig(0.05, 2.0, 1.5, 3.0));

    @Override
    public void initialize() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init(false);

        robotDrive = new RobotDriveSubsystem(robot);
        intake = new IntakeSubsystem(robot);
        transfer = new TransferSubsystem(robot);
        shooter = new ShooterSubsystem(robot);
        controls = new ControlSubsystem();
        limelight = new LimelightSubsystem(robot);
        limelight.init();
        targeting = new ShotTargeting(robot.odo, limelight);
        shooterRPM = new DistanceShooterRPM(targeting);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
        displayedCell = null;
        updateCellLeds();

        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Right stick X/Y moves and left stick X rotates; either driver can take over each axis.

        robotDrive.setDefaultCommand(new RobotDriveCommand(
                robotDrive,
                () -> drive.forward,
                () -> drive.strafe,
                this::selectTurn
        ));

        controls.configureControls(gamepad1Ex, gamepad2Ex, intake, transfer, shooter, shooterRPM, targeting);
    }

    @Override
    public void preRun() {
        targeting.initializeFieldPose();
        intake.setInitialServoPositions();
        shooter.closePollenGate();
        joystick.reset();
        driveTimer.reset();
    }

    @Override
    public void run() {
        targeting.beginLoop();
        double dt = driveTimer.seconds();
        driveTimer.reset();
        drive = joystick.update(
            -selectAxis(gamepad1.right_stick_y, gamepad2.right_stick_y),
            selectAxis(gamepad1.right_stick_x, gamepad2.right_stick_x),
            selectAxis(gamepad1.left_stick_x, gamepad2.left_stick_x),
            dt);
        super.run();
        updateCellLeds();

        telemetry.addData("Loop Frequency (Hz)", "%.1f", loopFrequencyHz(dt));
        telemetry.addLine("--------------------");
        telemetry.addData("Intake Motor Power", intake.getIntakeMotorPower());
        telemetry.addLine("--- SHOOTER TELEMETRY ---");
        telemetry.addData("Gamepad 2 right trigger", gamepad2.right_trigger);
        telemetry.addData("Shot mode", shooterRPM.getMode());
        telemetry.addData("RPM selection", shooterRPM.getStatus());
        telemetry.addData("Fixed manual RPM", "%.0f", DistanceShooterRPM.manualRPM);
        telemetry.addData("Alliance", targeting.getAlliance());
        telemetry.addData("Selected cell", targeting.getSelectedCell());
        telemetry.addData("Pinpoint targeting", targeting.getStatus());
        telemetry.addData("Chassis rotation", autoAimActive ? "Auto aim" : "Driver");
        telemetry.addData("Aim source", targeting.getAimSource());
        if (autoAimActive) {
            telemetry.addData("Aim error (deg)", "%.1f", targeting.getAimErrorDegrees());
        }
        telemetry.addData("Limelight tags", limelight.getTagDetected());
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
        telemetry.addLine("--- INTAKE & TRANSFER TELEMETRY ---");
        telemetry.addData("Feeder Configured RPM", "%.0f", intake.getFeederTargetRPM());
        telemetry.addData("Intake Actual RPM", "%.0f", intake.getFeederCurrentRPM());
        telemetry.addData("Transfer Target RPM", "%.0f", transfer.getFeederTargetRPM());
        telemetry.addData("Transfer Actual RPM", "%.0f", transfer.getTransferSpeedRPM());
        telemetry.update();
    }

    private void updateCellLeds() {
        ShotTargeting.Cell selectedCell = targeting.getSelectedCell();
        if (selectedCell == displayedCell) {
            return;
        }

        boolean left = selectedCell == ShotTargeting.Cell.LEFT;
        double red = left ? 0.0 : 1.0;
        double green = left ? 0.0 : 0.45;
        double blue = left ? 1.0 : 0.0;
        gamepad1.setLedColor(red, green, blue, Gamepad.LED_DURATION_CONTINUOUS);
        gamepad2.setLedColor(red, green, blue, Gamepad.LED_DURATION_CONTINUOUS);
        displayedCell = selectedCell;
    }

    @Override
    public void reset() {
        joystick.reset();
        if (limelight != null) {
            limelight.stop();
        }
        super.reset();
    }

    ///helper method to select the axis from either gamepad1 or gamepad2, prioritizing gamepad1 if it has a non-zero input.
    private static double selectAxis(double primary, double secondary) {
       return Math.abs(primary) > 0.05 ? primary : secondary;
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
        return drive.turn ;
    }

    private static double loopFrequencyHz(double elapsedSeconds) {
        return elapsedSeconds > 0.0 ? 1.0 / elapsedSeconds : 0.0;
    }


}
