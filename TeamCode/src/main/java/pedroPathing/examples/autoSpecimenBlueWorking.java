
package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import common.commands.arm.armPitchCommands.armPitchSpecimenDepositPos;
import common.commands.arm.armPitchCommands.armPitchSpecimenGrabPos;
import common.commands.arm.armPivotCommands.armPivotInitPos;
import common.commands.arm.armPivotCommands.armPivotSpecimenDepositPos;
import common.commands.arm.armPivotCommands.armPivotSpecimenGrabPos;
import common.commands.arm.armWristCommands.armWristCenterPos;
import common.commands.arm.armWristCommands.armWristSpecimenDepositPos;
import common.commands.follower.setMaxPower;
import common.commands.slide.setSlideTargetPosHor;
import common.commands.slidePivot.slidePivotSetPos;
import common.robot.robotHardware;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Config
@Autonomous(name = "Auto Specimen 5+0 Blue - WORKING", group = "BLUE AUTO")
public class autoSpecimenBlueWorking extends OpMode {
    robotHardware robot;
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean followPath1 = true;
    private boolean followPath2 = false;
    private boolean followPath3 = false;
    private boolean followPath4 = false; // New flag added
    private boolean followPath5 = false;
    private boolean followPath6 = false;
    private boolean followPath7 = false;
    private boolean followPath8 = false;
    private boolean followPath9 = false;
    private boolean followPath10 = false; // New flag added
    private boolean followPath11 = false;
    private boolean followPath12 = false;
    private boolean followPath13 = false;
    private boolean followPath14 = false;
    private boolean followPath15 = false;
    private boolean followPath16 = false;
    private boolean followPath17 = false;
    private boolean followPath18 = false;
    private boolean followPath19 = false;

    private Follower follower;

    private Path toDropZoneOne;
    private Path toSampleOne;
    private Path toSampleTwo;
    private Path toSampleThree;
    private Path toHumanFromSampleOne;
    private Path toHumanFromSampleTwo;
    private Path toHumanFromSampleThree;
    private Path toTwoSpecimenPick;
    private Path toTwoSpecimenDrop;
    private Path toTwoSpecimenDrop2;

    private Path toThreeSpecimenPick;
    private Path toThreeSpecimenDrop;

    private Path toFourSpecimenPick;
    private Path toFourSpecimenDrop;
    private Path toFiveSpecimenPick;
    private Path toFiveSpecimenDrop;
    private Path toPark;
    private Path toGrab;

    private BooleanSupplier slideRetracted = () -> robot.slideMotor.getPosition() <10 ;
    private BooleanSupplier pivotExtended = () -> robot.slidePivotMotor.getPosition() <500 ;



    @Override
    public void init() {
        robot = robotHardware.getInstance();
        robot.init(hardwareMap);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        // Path initialization remains the same
        toDropZoneOne = new Path(new BezierLine(
                new Point(0, 0, Point.CARTESIAN),
                new Point(21.3, 3.6, Point.CARTESIAN)));
        toDropZoneOne.setConstantHeadingInterpolation(0);

        toSampleOne = new Path(new BezierCurve(
                new Point(21.3, 3.6, Point.CARTESIAN),
                new Point(10, 3.6, Point.CARTESIAN),
                new Point(10, -26, Point.CARTESIAN),
                new Point(46, -26, Point.CARTESIAN),
                new Point(46, -38, Point.CARTESIAN)
        ));
        toSampleOne.setConstantHeadingInterpolation(0);

        toHumanFromSampleOne = new Path(new BezierLine(
                new Point(46, -38, Point.CARTESIAN),
                new Point(12, -38, Point.CARTESIAN)
        ));
        toHumanFromSampleOne.setConstantHeadingInterpolation(0);

        toSampleTwo = new Path(new BezierCurve(
                new Point(12, -38, Point.CARTESIAN),
                new Point(46, -38, Point.CARTESIAN),
                new Point(46, -46, Point.CARTESIAN)
        ));
        toSampleTwo.setConstantHeadingInterpolation(0);

        toHumanFromSampleTwo = new Path(new BezierLine(
                new Point(46, -46, Point.CARTESIAN),
                new Point(12, -46, Point.CARTESIAN)
        ));
        toHumanFromSampleTwo.setConstantHeadingInterpolation(0);


        toSampleThree = new Path(new BezierCurve(
                new Point(12, -46, Point.CARTESIAN),
                new Point(46, -46, Point.CARTESIAN),
                new Point(46, -54.5, Point.CARTESIAN)
        ));
        toSampleThree.setConstantHeadingInterpolation(0);

        toHumanFromSampleThree = new Path(new BezierLine(
                new Point(46, -54.5, Point.CARTESIAN),
                new Point(12, -54.5, Point.CARTESIAN)
        ));
        toHumanFromSampleThree.setConstantHeadingInterpolation(0);

        toTwoSpecimenPick = new Path(new BezierLine(
                new Point(12, -54.5, Point.CARTESIAN),
                new Point(4.65, -54.5, Point.CARTESIAN)
        ));
        toTwoSpecimenPick.setConstantHeadingInterpolation(0);

//        toTwoSpecimenDrop = new Path(new BezierCurve(
//                new Point(4.8, -54.5, Point.CARTESIAN),
//                new Point(7, -54.5, Point.CARTESIAN),
//                new Point(7, 8, Point.CARTESIAN),
//                new Point(25.5, 8, Point.CARTESIAN)
//        ));
//                toTwoSpecimenDrop.setConstantHeadingInterpolation(0);
        toTwoSpecimenDrop = new Path(new BezierLine(
                new Point(4.65, -54.5, Point.CARTESIAN),
                new Point(12, 8, Point.CARTESIAN)

        ));
        toTwoSpecimenDrop.setConstantHeadingInterpolation(0);
        toTwoSpecimenDrop2 = new Path(new BezierLine(
                new Point(12, 8, Point.CARTESIAN),
                new Point(25.5, 8, Point.CARTESIAN)
        ));
        toTwoSpecimenDrop2.setConstantHeadingInterpolation(0);

        toThreeSpecimenPick = new Path(new BezierLine(
                new Point(25.5, 8, Point.CARTESIAN),
                new Point(12, -30, Point.CARTESIAN)
        ));
        toThreeSpecimenPick.setConstantHeadingInterpolation(0);


        toThreeSpecimenDrop = new Path(new BezierCurve(
                new Point(4.8, -30, Point.CARTESIAN),
                new Point(16, 9, Point.CARTESIAN),
                new Point(25.5, 9, Point.CARTESIAN)
        ));
        toThreeSpecimenDrop.setConstantHeadingInterpolation(0);

        toFourSpecimenPick = new Path(new BezierLine(
                new Point(25.5, 9, Point.CARTESIAN),
                new Point(12, -30, Point.CARTESIAN)
        ));
        toFourSpecimenPick.setConstantHeadingInterpolation(0);

        toFourSpecimenDrop = new Path(new BezierCurve(
                new Point(4.8, -30, Point.CARTESIAN),
                new Point(16, 11, Point.CARTESIAN),
                new Point(25.5, 11, Point.CARTESIAN)
        ));
        toFourSpecimenDrop.setConstantHeadingInterpolation(0);

        toFiveSpecimenPick = new Path(new BezierLine(
                new Point(25.5, 11, Point.CARTESIAN),
                new Point(12, -30, Point.CARTESIAN)
        ));
        toFiveSpecimenPick.setConstantHeadingInterpolation(0);


        toFiveSpecimenDrop = new Path(new BezierCurve(
                new Point(4.8, -30, Point.CARTESIAN),
                new Point(16,12,Point.CARTESIAN),
                new Point(25.5, 12, Point.CARTESIAN)
        ));
        toFiveSpecimenDrop.setConstantHeadingInterpolation(0);

        toPark = new Path(new BezierLine(
                new Point(25.5, 12, Point.CARTESIAN),
                new Point(7, -30, Point.CARTESIAN)
        ));
        toPark.setConstantHeadingInterpolation(0);

        toGrab = new Path(new BezierLine(
                new Point(12, -30, Point.CARTESIAN),
                new Point(4.8, -30, Point.CARTESIAN)
        ));
        toGrab.setConstantHeadingInterpolation(0);

        CommandScheduler.getInstance().reset();

        follower.setMaxPower(1);


        // Remaining paths initialized as before


        follower.followPath(toDropZoneOne);
        new armClawGrabPos().schedule();
        new armPitchSpecimenDepositPos().schedule();
        new armPivotSpecimenDepositPos().schedule();
//        new armWristCenterPos().schedule();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("PLEASE READ: This autonomous has the CHASSIS MOVEMENT ONLY for the first 4 paths of the SPECIMEN AUTO" +
                "for the BLUE ALLIANCE. ONLY TILL SECOND SAMPLE ON SPIKE WITHOUT GETTING IT TO HUMAN PLAYER");
        telemetryA.update();
        follower.setMaxPower(1.0);

        robot.read();
        robot.periodic();
        robot.write();
        new slidePivotSetPos(400).schedule();
        new setSlideTargetPosHor(1200).schedule();
    }



    @Override
    public void loop() {
        robot.read();
        robot.periodic();
        robot.write();
        follower.update();
        follower.update();
        telemetry.addData("Current Path: ", follower.getCurrentPath());
        telemetry.update();
        CommandScheduler.getInstance().run();
        if (!follower.isBusy()) {
            if (followPath1) {
                follower.followPath(toSampleOne);
                new armClawReleasePos().schedule();
                new armPivotInitPos().schedule();
                new armPitchCenterPos().schedule();
                new armWristCenterPos().schedule();
                new setSlideTargetPosHor(0).schedule();
                new WaitUntilCommand(slideRetracted).schedule();
                followPath1 = false;
                followPath2 = true;
            } else if (followPath2) {
                follower.followPath(toHumanFromSampleOne);
                new slidePivotSetPos(0).schedule();
                followPath2 = false;
                followPath3 = true;
            } else if (followPath3) {
                follower.followPath(toSampleTwo);
                followPath3 = false;
                followPath4 = true;
            } else if (followPath4) {
                follower.followPath(toHumanFromSampleTwo);
                followPath4 = false;
                followPath5 = true;
            } else if (followPath5) {
                follower.followPath(toSampleThree);
                new slidePivotSetPos(1040).schedule();
                new armClawReleasePos().schedule();
                new armPitchSpecimenGrabPos().schedule();
                new armPivotSpecimenGrabPos().schedule();
                new armWristCenterPos().schedule();
                followPath5 = false;
                followPath6 = true;

            } else if (followPath6) {
                follower.followPath(toHumanFromSampleThree);
                followPath6 = false;
                followPath7 = true;
            } else if (followPath7) {
                follower.setMaxPower(1.0);
                follower.followPath(toTwoSpecimenPick);
                followPath7 = false;
                followPath8 = true;
            } else if (followPath8) {
//                new WaitCommand(75).schedule();
                new armClawGrabPos().schedule();
                follower.setMaxPower(1.0);
                follower.followPath(toTwoSpecimenDrop);
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                            new setSlideTargetPosHor(1200),
                            new slidePivotSetPos(400),
                            new WaitUntilCommand(pivotExtended),
                            new armPivotSpecimenDepositPos(),
                            new armPitchSpecimenDepositPos(),
                            new armWristSpecimenDepositPos()
                        )
                );
                followPath8 = false;
                followPath19 = true;
            }

            else if (followPath9) {

                follower.setMaxPower(0.75);
                follower.followPath(toThreeSpecimenPick);
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                            new armClawReleasePos(),
                            new armPitchCenterPos(),
                            new armPivotInitPos(),
                            new armWristCenterPos(),
                            new setSlideTargetPosHor(0),
                            new armPitchSpecimenGrabPos(),
                            new armPivotSpecimenGrabPos(),
                            new slidePivotSetPos(1040)
                    )
                );
                followPath9 = false;
                followPath10 = true;
            } else if (followPath10) {

                follower.setMaxPower(0.85);
                follower.followPath(toGrab);
                followPath10 = false;
                followPath11 = true;
            }
              else if (followPath11) {
                new armClawGrabPos().schedule();
                  follower.setMaxPower(1.0);
            follower.followPath(toThreeSpecimenDrop);
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new setSlideTargetPosHor(1200),
                                new slidePivotSetPos(400),
                                new WaitUntilCommand(pivotExtended),
                                new armPivotSpecimenDepositPos(),
                                new armPitchSpecimenDepositPos(),
                                new armWristSpecimenDepositPos()
                        )
                );
            followPath11 = false;
            followPath12 = true;
            } else if (followPath12) {
                follower.setMaxPower(0.75);
                follower.followPath(toFourSpecimenPick);
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new armClawReleasePos(),
                                new armPitchCenterPos(),
                                new armPivotInitPos(),
                                new armWristCenterPos(),
                                new setSlideTargetPosHor(0),
                                new armPitchSpecimenGrabPos(),
                                new armPivotSpecimenGrabPos(),
                                new slidePivotSetPos(1040)
                        )
                );
                followPath12 = false;
                followPath13 = true;
            } else if (followPath13) {

                follower.setMaxPower(0.85);
                follower.followPath(toGrab);
                followPath13 = false;
                followPath14 = true;
            } else if (followPath14) {
                new armClawGrabPos().schedule();
                  follower.setMaxPower(1.0);
                follower.followPath(toFourSpecimenDrop);
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new setSlideTargetPosHor(1200),
                                new slidePivotSetPos(400),
                                new WaitUntilCommand(pivotExtended),
                                new armPivotSpecimenDepositPos(),
                                new armPitchSpecimenDepositPos(),
                                new armWristSpecimenDepositPos()
                        )
                );
                followPath14 = false;
                followPath15 = true;
            } else if (followPath15) {
                follower.followPath(toFiveSpecimenPick);
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                                new armClawReleasePos(),
                                new armPitchCenterPos(),
                                new armPivotInitPos(),
                                new armWristCenterPos(),
                                new setSlideTargetPosHor(0),
                                new armPitchSpecimenGrabPos(),
                                new armPivotSpecimenGrabPos(),
                                new slidePivotSetPos(1040)
                        )
                );
                followPath15 = false;
                followPath16 = true;
            } else if (followPath16) {

                  follower.setMaxPower(0.85);
                follower.followPath(toGrab); // Last path
                followPath16 = false;
                followPath17 = true;
                // Optionally stop or loop back here
            }
            else if (followPath17) {
                new WaitCommand(100).schedule();
                new armClawGrabPos().schedule();
                new WaitCommand(100).schedule();
                follower.setMaxPower(1.0);
                follower.followPath(toFiveSpecimenDrop);
                new armPivotSpecimenDepositPos().schedule();
                new armPitchSpecimenDepositPos().schedule();
                new armWristSpecimenDepositPos().schedule();
                new slidePivotSetPos(400).schedule();
                new setSlideTargetPosHor(1200).schedule();
                followPath17 = false;
                followPath18 = true;
            } else if (followPath18) {
                follower.followPath(toPark); // Last path
                new armClawReleasePos().schedule();
                new setSlideTargetPosHor(0).schedule();
                new slidePivotSetPos(1040).schedule();
                new armPitchSpecimenGrabPos().schedule();
                new armPivotSpecimenGrabPos().schedule();
                new armWristCenterPos().schedule();
                followPath18 = false;
            }

            else if (followPath19) {
                follower.setMaxPower(1.0);
//                new armPivotSpecimenDepositPos().schedule();
//                new armPitchSpecimenDepositPos().schedule();
//                new armWristSpecimenDepositPos().schedule();
//                new WaitCommand(50).schedule();
//                new armClawGrabPos().schedule();
//                new WaitCommand(100).schedule();
                follower.followPath(toTwoSpecimenDrop2);
//                new armPivotSpecimenDepositPos().schedule();
//                new armPitchSpecimenDepositPos().schedule();
//                new armWristSpecimenDepositPos().schedule();
//                new slidePivotSetPos(400).schedule();
//                new setSlideTargetPosHor(1
//                ).schedule();
                followPath19 = false;
                followPath9 = true;




            }








            }

        }

    }
