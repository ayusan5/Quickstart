package pedroPathing.examples;//package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.pathgen.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import java.util.function.BooleanSupplier;

import common.commands.arm.armClawCommands.armClawGrabPos;
import common.commands.arm.armClawCommands.armClawReleasePos;
import common.commands.arm.armPitchCommands.armPitchCenterPos;
import common.commands.arm.armPitchCommands.armPitchSampleDepositPos;
import common.commands.arm.armPitchCommands.armPitchSampleGrabPos;
import common.commands.arm.armPitchCommands.armPitchSpecimenDepositPos;
import common.commands.arm.armPitchCommands.armPitchSpecimenGrabPos;
import common.commands.arm.armPivotCommands.armPivotInitPos;
import common.commands.arm.armPivotCommands.armPivotSampleGrabPos;
import common.commands.arm.armPivotCommands.armPivotSpecimenDepositPos;
import common.commands.arm.armPivotCommands.armPivotSpecimenGrabPos;
import common.commands.arm.armWristCommands.armWristCenterPos;
import common.commands.arm.armWristCommands.armWristSpecimenDepositPos;
import common.commands.follower.setMaxPower;
import common.commands.slide.setSlideTargetPosHor;
import common.commands.slide.setSlideTargetPosVer;
import common.commands.slidePivot.slidePivotSetPos;
import common.robot.robotHardware;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
//
//@Config
//@Autonomous(name = "Auto Sample 0+4 Blue - WORKING", group = "BLUE AUTO")
//public class autoSampleBlue extends OpMode {
//    robotHardware robot;
//    private Telemetry telemetryA;
//
//    public static double DISTANCE = 40;
//
//
//
//    private boolean followPath1 = true;
//    private boolean followPath2 = false;
//    private boolean followPath3 = false;
//    private boolean followPath4 = false; // New flag added
//    private boolean followPath5 = false;
//    private boolean followPath6 = false;
//    private boolean followPath7 = false;
////    private boolean followPath8 = false;
////    private boolean followPath9 = false;
////    private boolean followPath10 = false;
////    private boolean followPath11 = false;
////    private boolean followPath12 = false;
////    private boolean followPath13 = false;
////    private boolean followPath14 = false;
////    private boolean followPath15 = false;
////    private boolean followPath16 = false;
////    private boolean followPath17 = false;
////    private boolean followPath18 = false;
////    private boolean followPath19 = false;
//
//    private Follower follower;
//
//    private Path toDropPreload;
//    private Path toSampleOne;
//    private Path toSampleTwo;
//    private Path toSampleThree;
//    private Path toDropSampleOne;
//    private Path toDropSampleTwo;
//    private Path toDropSampleThree;
//    private Path toPark;
//     private BooleanSupplier slideSampleAfterDeposit = () -> robot.slideMotor.getPosition() < 10;
//    private BooleanSupplier slideRetracted = () -> robot.slideMotor.getPosition() < 10;
//    private BooleanSupplier slideExtended = ()  ->robot.slideMotor.getPosition()>2040;
//    private BooleanSupplier pivotUp = () -> robot.slidePivotMotor.getPosition() > 1000;
//    private BooleanSupplier pivotDown = () -> robot.slidePivotMotor.getPosition() < 50;
//
//
//    @Override
//    public void init() {
//        robot = robotHardware.getInstance();
//        robot.init(hardwareMap);
//        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
//
//        // Path initialization remains the same
//        toDropPreload = new Path(new BezierLine(
//                new Point(0, 0, Point.CARTESIAN),
//                new Point(-38.3, 11.7, Point.CARTESIAN)));
//        toDropPreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44));
//
//        toSampleOne = new Path(new BezierLine(
//                new Point(-38.3, 11.7, Point.CARTESIAN),
//                new Point(-32.4, 18, Point.CARTESIAN)
//        ));
//        toSampleOne.setConstantHeadingInterpolation(Math.toRadians(84));
//
//        toDropSampleOne = new Path(new BezierLine(
//                new Point(-32.4, 18, Point.CARTESIAN),
//                new Point(-38.3, 11.7, Point.CARTESIAN)
//        ));
//        toDropSampleOne.setConstantHeadingInterpolation(Math.toRadians(44));
//
//        toSampleTwo = new Path(new BezierLine(
//                new Point(-38.3, 11.7, Point.CARTESIAN),
//                new Point(-40.8, 18.4, Point.CARTESIAN)
//        ));
//        toSampleTwo.setConstantHeadingInterpolation(Math.toRadians(88));
//
//        toDropSampleTwo = new Path(new BezierLine(
//                new Point(-40.8, 18.4, Point.CARTESIAN),
//                new Point(-38.3, 11.7, Point.CARTESIAN)
//        ));
//        toDropSampleTwo.setConstantHeadingInterpolation(Math.toRadians(44));
//
//
//        toSampleThree = new Path(new BezierLine(
//                new Point(-38.3, 11.7, Point.CARTESIAN),
//                new Point(-44.1, 21, Point.CARTESIAN)
//        ));
//        toSampleThree.setConstantHeadingInterpolation(Math.toRadians(97.4));
//
//        toDropSampleThree = new Path(new BezierLine(
//                new Point(-44.1, 21, Point.CARTESIAN),
//                new Point(-38.3, 11.7, Point.CARTESIAN)
//        ));
//        toDropSampleThree.setConstantHeadingInterpolation(Math.toRadians(44));
//
//        toPark = new Path(new BezierCurve(
//                new Point(-38.3, 11.7, Point.CARTESIAN),
//                new Point(-30, 55, Point.CARTESIAN),
//                new Point(-20, 55, Point.CARTESIAN),
//                new Point(-6, 55, Point.CARTESIAN)
//        ));
//        toPark.setConstantHeadingInterpolation(Math.toRadians(0));
//
//
//        CommandScheduler.getInstance().reset();
//        follower.setMaxPower(1);
//        follower.followPath(toDropPreload);
//        CommandScheduler.getInstance().schedule(
//                new SequentialCommandGroup(
//                        new armClawGrabPos(),
//                        new armPitchCenterPos(),
//                        new armPivotInitPos(),
//                        new armWristCenterPos(),
//                        new slidePivotSetPos(1040),
//                        new WaitUntilCommand(pivotUp),
//                        new setSlideTargetPosHor(2050),
//                        new WaitUntilCommand(slideExtended),
//                        new armPitchSampleDepositPos()
//                )
//        );
//
//        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetryA.addLine("hi");
//        telemetryA.update();
//        follower.setMaxPower(0.75);
//
//        robot.read();
//        robot.periodic();
//        robot.write();
//    }
//
//
//    @Override
//    public void loop() {
//        robot.read();
//        robot.periodic();
//        robot.write();
//        follower.update();
//        follower.update();
//        telemetry.addData("Current Path: ", follower.getCurrentPath());
//        telemetry.update();
//        CommandScheduler.getInstance().run();
//        if (!follower.isBusy()) {
//            if (followPath1) {
//                follower.followPath(toSampleOne);
//
//                CommandScheduler.getInstance().schedule(
//                        new SequentialCommandGroup(
//                                new armClawReleasePos(),
//                                new armPitchCenterPos(),
//                                new armWristCenterPos(),
//                                new armPivotInitPos(),
//                                new setSlideTargetPosVer(0),
//                                new WaitUntilCommand(slideRetracted),
//                                new slidePivotSetPos(0),
//                                new WaitUntilCommand(pivotDown),
//                                new setSlideTargetPosHor(1500),
//                                new WaitUntilCommand(slideExtended),
//                                new armPivotSampleGrabPos(),
//                                new armPitchSampleGrabPos()
//                        ));
//                followPath1 = false;
//                followPath2 = true;
//            } else if (followPath2) {
//                follower.followPath(toDropSampleOne);
//                CommandScheduler.getInstance().schedule(
//                        new SequentialCommandGroup(
//                                new armClawGrabPos(),
//                                new armPitchCenterPos(),
//                                new armPivotInitPos(),
//                                new armWristCenterPos(),
//                                new setSlideTargetPosVer(0),
//                                new WaitUntilCommand(slideRetracted),
//                                new slidePivotSetPos(1040),
//                                new WaitUntilCommand(pivotUp),
//                                new setSlideTargetPosHor(2050),
//                                new WaitUntilCommand(slideExtended),
//                                new armPitchSampleDepositPos()
//                        ));
//                followPath2 = false;
//                followPath3 = true;
//            } else if (followPath3) {
//                follower.followPath(toSampleTwo);
//                CommandScheduler.getInstance().schedule(
//                        new SequentialCommandGroup(
//                                new armClawReleasePos(),
//                                new armPitchCenterPos(),
//                                new armWristCenterPos(),
//                                new armPivotInitPos(),
//                                new setSlideTargetPosVer(0),
//                                new WaitUntilCommand(slideRetracted),
//                                new slidePivotSetPos(0),
//                                new WaitUntilCommand(pivotDown),
//                                new setSlideTargetPosHor(1500),
//                                new WaitUntilCommand(slideExtended),
//                                new armPivotSampleGrabPos(),
//                                new armPitchSampleGrabPos()
//                        ));
//                followPath3 = false;
//                followPath4 = true;
//            } else if (followPath4) {
//                follower.followPath(toDropSampleTwo);
//                CommandScheduler.getInstance().schedule(
//                        new SequentialCommandGroup(
//                                new armClawGrabPos(),
//                                new armPitchCenterPos(),
//                                new armPivotInitPos(),
//                                new armWristCenterPos(),
//                                new setSlideTargetPosVer(0),
//                                new WaitUntilCommand(slideRetracted),
//                                new slidePivotSetPos(1040),
//                                new WaitUntilCommand(pivotUp),
//                                new setSlideTargetPosHor(2050),
//                                new WaitUntilCommand(slideExtended),
//                                new armPitchSampleDepositPos()
//                        ));
//                followPath4 = false;
//                followPath5 = true;
//            } else if (followPath5) {
//                follower.followPath(toSampleThree);
//                CommandScheduler.getInstance().schedule(
//                        new SequentialCommandGroup(
//                                new armClawReleasePos(),
//                                new armPitchCenterPos(),
//                                new armWristCenterPos(),
//                                new armPivotInitPos(),
//                                new setSlideTargetPosVer(0),
//                                new WaitUntilCommand(slideRetracted),
//                                new slidePivotSetPos(0),
//                                new WaitUntilCommand(pivotDown),
//                                new setSlideTargetPosHor(1500),
//                                new WaitUntilCommand(slideExtended),
//                                new armPivotSampleGrabPos(),
//                                new armPitchSampleGrabPos()
//                        ));
//                followPath5 = false;
//                followPath6 = true;
//            } else if (followPath6) {
//                follower.followPath(toDropSampleThree);
//                CommandScheduler.getInstance().schedule(
//                        new SequentialCommandGroup(
//                                new armClawGrabPos(),
//                                new armPitchCenterPos(),
//                                new armPivotInitPos(),
//                                new armWristCenterPos(),
//                                new setSlideTargetPosVer(0),
//                                new WaitUntilCommand(slideRetracted),
//                                new slidePivotSetPos(1040),
//                                new WaitUntilCommand(pivotUp),
//                                new setSlideTargetPosHor(2050),
//                                new WaitUntilCommand(slideExtended),
//                                new armPitchSampleDepositPos()
//                        ));
//                followPath6 = false;
//                followPath7 = true;
//            } else if (followPath7) {
//                follower.followPath(toPark);
//                CommandScheduler.getInstance().schedule(
//                        new SequentialCommandGroup(
//                                new armClawReleasePos(),
//                                new armPitchCenterPos(),
//                                new armWristCenterPos(),
//                                new armPivotInitPos(),
//                                new setSlideTargetPosVer(0),
//                                new WaitUntilCommand(slideRetracted),
//                                new slidePivotSetPos(0)
////                                new WaitUntilCommand(pivotDown),
////                                new setSlideTargetPosHor(1500),
////                                new WaitUntilCommand(slideExtended),
////                                new armPivotSampleGrabPos(),
////                                new armPitchSampleGrabPos()
//                        ));
//                followPath7 = false;
//            }
//        }
//    }
//}
//
//
//
//package pedroPathing.examples;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.command.Command;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import common.commands.arm.armClawCommands.armClawGrabPos;
import common.commands.arm.armClawCommands.armClawReleasePos;
import common.commands.arm.armPitchCommands.armPitchCenterPos;
import common.commands.arm.armPitchCommands.armPitchSampleDepositPos;
import common.commands.arm.armPitchCommands.armPitchSampleGrabPos;
import common.commands.arm.armPivotCommands.armPivotInitPos;
import common.commands.arm.armPivotCommands.armPivotSampleGrabPos;
import common.commands.arm.armWristCommands.armWristCenterPos;
import common.commands.slide.setSlideTargetPosHor;
import common.commands.slide.setSlideTargetPosVer;
import common.commands.slidePivot.slidePivotSetPos;
import common.robot.robotHardware;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "Auto Sample 0+4 Blue - WORKING", group = "BLUE AUTO")
public class autoSampleBlue extends OpMode {
    robotHardware robot;
    private Telemetry telemetryA;
    private Follower follower;

    private Path toDropPreload, toSampleOne, toDropSampleOne,
            toSampleTwo, toDropSampleTwo, toSampleThree, toDropSampleThree, toPark;


    private BooleanSupplier slideRetracted = () -> robot.slideMotor.getPosition() < 10;
    private BooleanSupplier slideExtended = ()  ->robot.slideMotor.getPosition()>2000;
    private BooleanSupplier slideHorExt = ()  ->robot.slideMotor.getPosition()>1400;

    private BooleanSupplier pivotUp = () -> robot.slidePivotMotor.getPosition() > 1000;
    private BooleanSupplier pivotDown = () -> robot.slidePivotMotor.getPosition() < 50;
//    private BooleanSupplier slideRetracted, slideExtended, pivotUp, pivotDown;

    @Override
    public void init() {
        robot = robotHardware.getInstance();
        robot.init(hardwareMap);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        initializePaths();
//        setupBooleanSuppliers();
        setupCommandScheduler();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.read();
        robot.periodic();
        robot.write();
    }

    private void initializePaths() {
        // Preload drop path
        toDropPreload = new Path(new BezierLine(
                new Point(0, 0, Point.CARTESIAN),
                new Point(-38.5, 8, Point.CARTESIAN)));
        toDropPreload.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44));

        // Sample one paths
        toSampleOne = new Path(new BezierLine(
                new Point(-38.5, 8, Point.CARTESIAN),
                new Point(-32.3, 20, Point.CARTESIAN)));
        toSampleOne.setConstantHeadingInterpolation(Math.toRadians(90));

        toDropSampleOne = new Path(new BezierLine(
                new Point(-32.3, 20, Point.CARTESIAN),
                new Point(-38.5, 8, Point.CARTESIAN)));
        toDropSampleOne.setConstantHeadingInterpolation(Math.toRadians(44));

        // Sample two paths
        toSampleTwo = new Path(new BezierLine(
                new Point(-38.5, 8, Point.CARTESIAN),
                new Point(-39, 19.7, Point.CARTESIAN)));
        toSampleTwo.setConstantHeadingInterpolation(Math.toRadians(90));

        toDropSampleTwo = new Path(new BezierLine(
                new Point(-39, 19.7, Point.CARTESIAN),
                new Point(-38.5, 8, Point.CARTESIAN)));
        toDropSampleTwo.setConstantHeadingInterpolation(Math.toRadians(44));

        // Sample three paths
        toSampleThree = new Path(new BezierLine(
                new Point(-38.5, 8, Point.CARTESIAN),
                new Point(-44.5, 26.5, Point.CARTESIAN)));
        toSampleThree.setLinearHeadingInterpolation(Math.toRadians(44),Math.toRadians(120));

        toDropSampleThree = new Path(new BezierLine(
                new Point(-44.5, 26.5, Point.CARTESIAN),
                new Point(-39, 8, Point.CARTESIAN)));
        toDropSampleThree.setConstantHeadingInterpolation(Math.toRadians(44));

        // Park path
        toPark = new Path(new BezierCurve(
                new Point(-39, 8, Point.CARTESIAN),
                new Point(-30, 55, Point.CARTESIAN),
                new Point(-20, 55, Point.CARTESIAN),
                new Point(-14, 55, Point.CARTESIAN)));
        toPark.setConstantHeadingInterpolation(Math.toRadians(0));
    }

//    private void setupBooleanSuppliers() {
//        slideRetracted = () -> robot.slideMotor.getPosition() < 10;
//        slideExtended = () -> robot.slideMotor.getPosition() > 2040;
//        pivotUp = () -> robot.slidePivotMotor.getPosition() > 1000;
//        pivotDown = () -> robot.slidePivotMotor.getPosition() < 50;
//        slideHorExt = () -> robot.slideMotor.getPosition()>1400;
//    }

    private void setupCommandScheduler() {
        CommandScheduler.getInstance().reset();
        follower.setMaxPower(0.65);

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                 new SequentialCommandGroup(
                                        new armClawGrabPos(),
                                        new armPitchCenterPos(),
                                        new armPivotInitPos(),
                                        new armWristCenterPos(),
                                        new slidePivotSetPos(1040),
                                        new WaitUntilCommand(pivotUp),
                                        new setSlideTargetPosHor(2050),
                                        new WaitUntilCommand(slideExtended),
                                        new armPitchSampleDepositPos()
                                        ), new FollowPathCommand(follower, toDropPreload)
                                        ),


                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new armClawReleasePos(),
                                        new armPitchCenterPos(),
                                        new armWristCenterPos(),
                                        new armPivotInitPos(),
                                        new setSlideTargetPosVer(0),
                                        new WaitUntilCommand(slideRetracted),
                                        new slidePivotSetPos(0),
                                        new WaitUntilCommand(pivotDown),
                                        new armPitchSampleGrabPos(),
                                        new setSlideTargetPosHor(1540),
//                                        new WaitUntilCommand(slideHorExt),
                                        new WaitCommand(1200),
//                                        new armPitchSampleGrabPos(),
//                                        new WaitCommand(200),
                                        new armPivotSampleGrabPos(),
                                        new WaitCommand(300),
                                        new armClawGrabPos(),
                                        new WaitCommand(200)
                                ),
                                new FollowPathCommand(follower, toSampleOne))

                ),

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new armPitchCenterPos(),
                                        new armPivotInitPos(),
                                        new armWristCenterPos(),
                                        new setSlideTargetPosVer(0),
                                        new WaitUntilCommand(slideRetracted),
                                        new slidePivotSetPos(1040),
                                        new WaitUntilCommand(pivotUp),
                                        new setSlideTargetPosHor(2050),
                                        new WaitUntilCommand(slideExtended),
                                        new armPitchSampleDepositPos(),
                                        new armClawReleasePos()
                                ),
                                new FollowPathCommand(follower, toDropSampleOne)
                        )),

                new SequentialCommandGroup(
                new ParallelCommandGroup(
                                new SequentialCommandGroup(

                                        new armClawReleasePos(),
                                        new armPitchCenterPos(),
                                        new armWristCenterPos(),
                                        new armPivotInitPos(),
                                        new setSlideTargetPosVer(0),
                                        new WaitUntilCommand(slideRetracted),
                                        new slidePivotSetPos(0),
                                        new WaitUntilCommand(pivotDown),
                                        new armPitchSampleGrabPos(),
                                        new setSlideTargetPosHor(1540),
//                                        new WaitUntilCommand(slideHorExt),
                                        new WaitCommand(1200),
//                                        new armPitchSampleGrabPos(),
//                                        new WaitCommand(200),
                                        new armPivotSampleGrabPos(),
                                        new WaitCommand(300),
                                        new armClawGrabPos(),
                                        new WaitCommand(200)
                                ),
                        new FollowPathCommand(follower, toSampleTwo))),

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new armClawGrabPos(),
                                        new armPitchCenterPos(),
                                        new armPivotInitPos(),
                                        new armWristCenterPos(),
                                        new setSlideTargetPosVer(0),
                                        new WaitUntilCommand(slideRetracted),
                                        new slidePivotSetPos(1040),
                                        new WaitUntilCommand(pivotUp),
                                        new setSlideTargetPosHor(2050),
                                        new WaitUntilCommand(slideExtended),
                                        new armPitchSampleDepositPos()
                                ),
                                new FollowPathCommand(follower, toDropSampleTwo)
                        )),

                new SequentialCommandGroup(
                new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new armClawReleasePos(),
                                        new armPitchCenterPos(),
                                        new armWristCenterPos(),
                                        new armPivotInitPos(),
                                        new setSlideTargetPosVer(0),
                                        new WaitUntilCommand(slideRetracted),
                                        new slidePivotSetPos(0),
                                        new WaitUntilCommand(pivotDown),
                                        new armPitchSampleGrabPos(),
                                        new setSlideTargetPosHor(1540),
//                                        new WaitUntilCommand(slideHorExt),
                                        new WaitCommand(1200),
//                                        new armPitchSampleGrabPos(),
//                                        new WaitCommand(200),
                                        new armPivotSampleGrabPos(),
                                        new WaitCommand(300),
                                        new armClawGrabPos(),
                                        new WaitCommand(200)
                                ),
                                new FollowPathCommand(follower, toSampleThree)
                        )),

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new armClawGrabPos(),
                                        new armPitchCenterPos(),
                                        new armPivotInitPos(),
                                        new armWristCenterPos(),
                                        new setSlideTargetPosVer(0),
                                        new WaitUntilCommand(slideRetracted),
                                        new slidePivotSetPos(1040),
                                        new WaitUntilCommand(pivotUp),
                                        new setSlideTargetPosHor(2050),
                                        new WaitUntilCommand(slideExtended),
                                        new armPitchSampleDepositPos()

                                ),
                                new FollowPathCommand(follower, toDropSampleThree)
                        )),

                new SequentialCommandGroup(
                new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new armClawReleasePos(),
                                        new setSlideTargetPosVer(0),
                                        new WaitUntilCommand(slideRetracted),
                                        new slidePivotSetPos(0),
                                        new WaitUntilCommand(pivotDown)
                                ),
                        new SequentialCommandGroup(
                                new FollowPathCommand(follower, toPark),
                                new setSlideTargetPosVer(500),
                                 new slidePivotSetPos(450))

                        ),

                        new InstantCommand(() -> telemetry.addLine("Parking complete")))));
    }

    @Override
    public void loop() {
        robot.read();
        robot.periodic();
        robot.write();
        CommandScheduler.getInstance().run();

        telemetry.addData("Current Path", follower.getCurrentPath());
        telemetry.addData("Slide Position", robot.slideMotor.getPosition());
        telemetry.addData("Pivot Position", robot.slidePivotMotor.getPosition());
        telemetry.addData("Command Status", CommandScheduler.getInstance().isScheduled() ? "Running" : "Idle");
        telemetry.update();
    }


    private class FollowPathCommand extends CommandBase {
        private final Follower follower;
        private final Path path;
        private boolean isFinished = false;

        public FollowPathCommand(Follower follower, Path path) {
            this.follower = follower;
            this.path = path;
        }

        @Override
        public void initialize() {
            isFinished = false;
            follower.followPath(path);
        }

        @Override
        public void execute() {
            follower.update();
            isFinished = !follower.isBusy();
        }

        @Override
        public boolean isFinished() {
            return isFinished;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                // Handle interruption if needed
                telemetry.addLine("Path interrupted!");
            }
        }
    }

}






