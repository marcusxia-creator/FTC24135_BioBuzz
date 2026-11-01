package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.src.Commands.RobotDriveCommand;
import org.firstinspires.ftc.teamcode.src.Commands.ShooterCommand;
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

        // Drive with whichever gamepad joystick is actively being moved
        robotDrive.setDefaultCommand(new RobotDriveCommand(
                robotDrive,
                () -> {
                    if (Math.abs(gamepad_1.getRightY()) > 0.05) return -gamepad_1.getRightY();
                    return -gamepad_2.getRightY();
                },
                () -> {
                    if (Math.abs(gamepad_1.getRightX()) > 0.05) return gamepad_1.getRightX();
                    return gamepad_2.getRightX();
                },
                () -> {
                    if (Math.abs(gamepad_1.getLeftX()) > 0.05) return gamepad_1.getLeftX();
                    return gamepad_2.getLeftX();
                }
        ));

        ///------------------Gamepad 1 & Gamepad 2 Controls-----------------------
        ///Run intake (DPAD_RIGHT on gamepad_1 OR gamepad_2)
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

        new GamepadButton(gamepad_2, GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(new InstantCommand(() -> {
                    if (gamepad_2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        intake.runRollersNectar();
                    } else if (gamepad_2.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                        intake.runRollersFlower();
                    } else {
                        intake.runRollersPollen();
                    }
                }, intake));

        // Reverse Intake (DPAD_LEFT alone) or Stop Intake (DPAD_LEFT + RIGHT_BUMPER)
        new GamepadButton(gamepad_1, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> {
                    if (gamepad_1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        intake.reverseIntake();
                    } else {
                        intake.stop();
                    }
                }, intake));

        new GamepadButton(gamepad_2, GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(new InstantCommand(() -> {
                    if (gamepad_2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                        intake.reverseIntake();
                    } else {
                        intake.stop();
                    }
                }, intake));

        ///RUN SHOOTER (Gamepad 1 OR Gamepad 2)
        new GamepadButton(gamepad_1, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(shooter::runFlywheel, shooter));
        new GamepadButton(gamepad_2, GamepadKeys.Button.X)
                .whenPressed(new InstantCommand(shooter::runFlywheel, shooter));

        new GamepadButton(gamepad_1, GamepadKeys.Button.Y)
                .whenPressed(new ShooterCommand(shooter));
        new GamepadButton(gamepad_2, GamepadKeys.Button.Y)
                .whenPressed(new ShooterCommand(shooter));

        // SHOOTER SPEED ADJUSTMENT (Diagonal Misfire Guard: ignores UP/DOWN if LEFT or RIGHT is pressed)
        new GamepadButton(gamepad_1, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (!gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && !gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                        shooter.increasePower();
                    }
                }, shooter));

        new GamepadButton(gamepad_1, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (!gamepad_1.getButton(GamepadKeys.Button.DPAD_LEFT) && !gamepad_1.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                        shooter.decreasePower();
                    }
                }, shooter));

        // INTAKE SPEED ADJUSTMENT (Gamepad 2)
        new GamepadButton(gamepad_2, GamepadKeys.Button.DPAD_UP)
                .whenPressed(new InstantCommand(() -> {
                    if (!gamepad_2.getButton(GamepadKeys.Button.DPAD_LEFT) && !gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                        intake.increasePower();
                    }
                }, intake));

        new GamepadButton(gamepad_2, GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(new InstantCommand(() -> {
                    if (!gamepad_2.getButton(GamepadKeys.Button.DPAD_LEFT) && !gamepad_2.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
                        intake.decreasePower();
                    }
                }, intake));

        ///STOP ALL (Gamepad 1 OR Gamepad 2)
        new GamepadButton(gamepad_1, GamepadKeys.Button.B)
                .whenPressed(new InstantCommand(() -> {
                    shooter.stop();
                    intake.stop();
                }, shooter, intake));

        new GamepadButton(gamepad_2, GamepadKeys.Button.B)
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
