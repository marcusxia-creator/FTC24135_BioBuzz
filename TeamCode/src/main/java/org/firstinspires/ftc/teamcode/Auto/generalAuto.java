package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions.ActionParallel.TERMINATIONTYPE.*;
import static org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.Units.Unit.*;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.flowerRampUpPos;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.gateUpPos;
import static org.firstinspires.ftc.teamcode.src.UniversalTools.RobotActionConfig.pollenDownPos;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing.ArcDirection;
import org.firstinspires.ftc.teamcode.src.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.src.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.src.UniversalTools.RobotHardware;
import org.firstinspires.ftc.teamcode.lib.CommandBase.Action;
import org.firstinspires.ftc.teamcode.lib.CommandBase.PrebuiltActions.*;
import org.firstinspires.ftc.teamcode.lib.CommandBase.ScheduledOpMode;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.IceWaddler;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.SpecialMeasurements.*;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Math.Measurement.*;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing.PrebuiltMotionProfiles.*;
import org.firstinspires.ftc.teamcode.lib.IceWaddler2.src.Pathing.PrebuiltPathingElements.*;

@Config
@Autonomous(name="Auto (Both Sides)",preselectTeleOp="Basic TeleOp")
public class generalAuto extends ScheduledOpMode {
    RobotHardware robot;
    IceWaddler waddler;

    Action path;

    IntakeSubsystem intake;
    ShooterSubsystem shooter;

    //Pathing Positions. All positions are in x,y, as on a graph, and headings are clockwise from vertical
    public static Position startPoint=new Position(new Vector(5.15,-1.40,feet),new NormalizedAngle(-90,deg));//Position at which the robot starts. I think this position's actually illegal rn
    public static Position leftHive=new Position(new Vector(5,-1.5,feet),new NormalizedAngle(-90,deg));   //The position that the robot goes to to shoot into the left hive. This is the first hive that the robot visits
    public static Position rightHive=new Position(new Vector(-4.5,-1,feet),new NormalizedAngle(90,deg));  //The position that the robot goes to to shoot into the right hive.
    public static Position flower1=new Position(new Vector(2,-4.6,feet),new NormalizedAngle(180,deg));  //The position that the robot goes to intake from the first flower. This is on the alliance side wall
    public static Position prep_flower2=new Position(new Vector(-3.5,-2,feet),new NormalizedAngle(-90,deg));
    public static Position flower2=new Position(new Vector(-4.6,-2,feet),new NormalizedAngle(-90,deg)); //The position that the robot goes to intake from the second flower. This is the flower closest to the right hive.
    public static Position parkPoint=new Position(new Vector(-3.8,-5,feet),new NormalizedAngle(-90,deg));//This is the position at which the robot parks at the end of auto

    public static Scalar arcRadius=new Scalar(2.5,feet);//The radius of the arc that the robot drives when moving across the field to shoot at the right hive for the first time

    //Timings
    public static Scalar shootTime=new Scalar(2.5,s);//Amount of time needed to consistently shoot all four pollen
    public static Scalar intakeTime=new Scalar(2,s);//Amount of time to consistently intake all four pollen from flower

    @Override
    public void init() {
        robot=new RobotHardware(hardwareMap);
        robot.init();

        waddler=new IceWaddler(robot.driveTrain, robot.localizer);
        waddler.init(startPoint,false);

        intake=new IntakeSubsystem(robot);
        shooter=new ShooterSubsystem(robot);

        //All just inits, like in teleop

        //sofstart ramp servos
        robot.leftFlowerRampServo.setPosition(flowerRampUpPos);
        robot.rightFlowerRampServo.setPosition(flowerRampUpPos);
        robot.pollenLimitingServo.setPosition(pollenDownPos);
        robot.gateServo.setPosition(gateUpPos);

        //The path is an action series, which starts at action 1 and moves to the next as soon as the current action is complete

        path=new ActionSeries(
                waddler.new InitPath(new PathingPoint(startPoint)),//This sets the first point in the path

                //move and shoot in left hive
                //This actionParallel moves toward the left hive shooting position while simultaneously spooling up the flywheel. The `FIRST` termination type means that this ActionParellel ends and moves to the next action when the robot reaches the end point and the `motionaction` ends
                new ActionParallel(FIRST,
                        waddler.new MotionAction(new Line(new PathingPoint(leftHive), new maxSpeedMP(), new linearHP(), new String[]{})),//Drives in a straight line towards the shooting position. This is the terminating action
                        shooter.new spoolFlywheelAction()),//This action spools up the flywheel without shooting
                //Holds robot still for 2 seconds to spool flywheel
                new ActionParallel(FIRST,
                        new Delay(4),
                        waddler.new MotionAction(new holdPos(new String[]{})),
                        shooter.new spoolFlywheelAction()
                ),
                //This actionParallel simultaneously runs the shooter while holding the robot still for the defined shootTime. The FIRST termination type means that this action ends when the shooting delay ends
                new ActionParallel(FIRST,
                        new Delay(shootTime),shooter.new ShootAction(),
                        waddler.new MotionAction(new holdPos(new String[]{}))),//Holds the robot still

                //Move and intake from the first flower
                //This ActionParallel drives in a straight line towards the first flower, and runs the intake to pre-extend the ramp. The `ALL` termination type means the action ends when all actions are complete
                new ActionParallel(ALL,
                        intake.new flowerIntakeAction(),//Run intake, mostly to drop the ramp
                        waddler.new MotionAction(new Line(new PathingPoint(flower1), new maxSpeedMP(), new linearHP(), new String[]{}))),//Drive toward the flower
                //This action runs the intake while holding the robot still for the defined intakeTime
                new ActionParallel(FIRST,
                        new Delay(intakeTime),intake.new flowerIntakeAction(),
                        waddler.new MotionAction(new holdPos(new String[]{}))),//Holds robot still

                //Drive toward right hive and shoot
                //This actionParallel drives toward the shooting position for the right hive, begin to spool the flywheel, and continues to run the intake to keep the ramp down and avoid flipping the robot against the flower
                //The endpoint is defined such that it ends at the start of the next arc, the positions of which will be explained later
                //Note that this is a continuous movement, such that this motion flows into the next arc without stopping, and instead moves at a constant 1m/s when crossing through the endpoint
                //                                                                                                                                               this speed is defined here ⌄ ⌄ ⌄ ⌄ ⌄ ⌄ ⌄ ⌄ ⌄ ⌄ ⌄ ⌄ ⌄ ⌄ ⌄ ⌄ ⌄ ⌄
                new ActionParallel(FIRST,
                        waddler.new MotionAction(new Line(new PathingPoint(new Position(new Vector(rightHive.getX().add(arcRadius), rightHive.getY().sub(arcRadius)),rightHive.getHeading()),new Scalar(1,metersPerSecond)), new maxSpeedMP(), new linearHP(), new String[]{})),
                        intake.new flowerIntakeAction(),//Continue running the intake. The intake will stop and the ramp will retract at the end of the actionParallel
                        shooter.new spoolFlywheelAction()),
                //This action spools the flywheel while moving in an arc to the shooting position
                //the radius for the arc is defined above as `arcRadius`. It covers an angle of 90°, and ends at the right hive shooting pos. It should flow pretty smoothly from the last movement. Note that FixedArcs are defined by the center and angle of sweep. The movement is also restricted to a max speed of 1m/s
                new ActionParallel(ALL,shooter.new spoolFlywheelAction(),
                        waddler.new MotionAction(new FixedArc(new Vector(rightHive.getX().add(arcRadius),rightHive.getY()),new PositiveAngle(90,deg), ArcDirection.CLOCKWISE,new Scalar(0,metersPerSecond),rightHive.getHeading(), new maxSpeedMP(new Scalar(1,metersPerSecond)), new linearHP(), new String[]{}))),
                //As before, this actionParallel simultaneously runs the shooter while holding the robot still for the shooting duration
                new ActionParallel(FIRST,new Delay(shootTime),shooter.new ShootAction(),
                        waddler.new MotionAction(new holdPos(new String[]{}))),

                //Move and intake from the second flower
                //As before, this action moves toward the flower in a straight line while running the intake to lower the ramp
                new ActionParallel(ALL,
                        waddler.new MotionAction(new Line(new PathingPoint(prep_flower2), new maxSpeedMP(), new linearHP(), new String[]{}))),
                new ActionParallel(ALL,
                        intake.new flowerIntakeAction(),
                        waddler.new MotionAction(new Line(new PathingPoint(flower2), new maxSpeedMP(), new linearHP(), new String[]{}))),
                new ActionParallel(FIRST,
                        new Delay(intakeTime),intake.new flowerIntakeAction(),
                        waddler.new MotionAction(new holdPos(new String[]{}))),

                //Move to the right hive shooting pos and shoot
                new ActionParallel(ALL,
                        shooter.new spoolFlywheelAction(),
                        waddler.new MotionAction(new Line(new PathingPoint(rightHive), new maxSpeedMP(), new linearHP(), new String[]{})),
                        intake.new flowerIntakeAction()),
                new ActionParallel(FIRST,
                        new Delay(shootTime),shooter.new ShootAction(),
                        waddler.new MotionAction(new holdPos(new String[]{}))),

                //Move to park
                waddler.new MotionAction(new Line(new PathingPoint(parkPoint), new maxSpeedMP(), new linearHP(), new String[]{}))
        );

        //Register the path into the root action
        rootAction=new ActionParallel(NONE,
                path//,telemetry or smth, if needed
        );
    }
}
