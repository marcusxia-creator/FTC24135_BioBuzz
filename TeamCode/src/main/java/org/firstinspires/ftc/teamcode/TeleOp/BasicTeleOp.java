package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.src.Commands.RobotDriveCommand;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.RobotDriveSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;

@TeleOp (name = "Basic TeleOp", group = "firstinspireslinkthingCHANGE")
public class BasicTeleOp extends CommandOpMode {
    private RobotDriveSubsystem robotDrive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private GamepadEx gamepad_1;
    private GamepadEx gamepad_2;

    @Override
    public void initialize() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        robot.init();

        robotDrive = new RobotDriveSubsystem(robot);
        intake = new IntakeSubsystem(robot);
        shooter = new ShooterSubsystem(robot);

        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);

        robotDrive.setDefaultCommand(new RobotDriveCommand(
                robotDrive,
                () -> -gamepad_1.getRightY(),
                () -> gamepad_1.getRightX(),
                () -> gamepad_1.getLeftX()
        ));

        ///------------------Gamepad-----------------------
        ///Run intake
        // 1. DPAD_RIGHT + RIGHT_BUMPER = Nectar Intaking
        // 2. DPAD_RIGHT + LEFT_BUMPER  = Flower Intaking
        // 3. DPAD_RIGHT alone          = Pollen Intaking
        new GamepadButton(gamepad_1, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> {
                    if (gamepad_1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        intake.runRollersNectar();
                    } else if (gamepad_1.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                        intake.runRollersFlower();
                    } else {
                        intake.runRollersPollen();
                    }
                }, intake));

        // 4. DPAD_LEFT = Stop Intaking
        new GamepadButton(gamepad_1, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(intake::stop, intake));

        ///RUN SHOOTER
        new GamepadButton(gamepad_1, GamepadKeys.Button.X)
                .whenPressed(new RunCommand(shooter::runFlywheel, shooter));

        // SHOOTER SPEED ADJUSTMENT (Gamepad 1: DPAD_UP / DPAD_DOWN)
        new GamepadButton(gamepad_1, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(shooter::increasePower, shooter));

        new GamepadButton(gamepad_1, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(shooter::decreasePower, shooter));

        // INTAKE SPEED ADJUSTMENT (Gamepad 2: DPAD_UP / DPAD_DOWN)
        new GamepadButton(gamepad_2, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(intake::increasePower, intake));

        new GamepadButton(gamepad_2, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(intake::decreasePower, intake));

        ///STOP ALL
        new GamepadButton(gamepad_1, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    shooter.stop();
                    intake.stop();
                }, shooter, intake));
    }

    @Override
    public void run() {
        super.run();
        telemetry.addData("Intake Motor Power", intake.getIntakeMotorPower());
        telemetry.addData("Shooter Velocity", shooter.getShooterVelocity());
        telemetry.addData("Target Velocity", shooter.getTargetVelocity());
        telemetry.update();
    }
}
