/** DO NOT TOUCH */

package pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.pathgen.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "Auto Specimen 5+0 Blue - WIP", group = "BLUE AUTO")
@Disabled
public class AutoSpecimenBlue extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean followPathTwo = true;
    private boolean followPathThree = false;
    private boolean followPathFour = false;
    private boolean followPathFive = false;
    private boolean followPathSix = false;
    private boolean followPathSeven = false;
    private boolean followPathEight = false;
    private boolean followPathNine = false;
    private boolean followPathTen = false;
    private boolean followPathEleven = false;
    private boolean followPathTwelve = false;
    private boolean followPathThirteen = false;

    private Follower follower;

    private Path toDropZoneOne;
    private Path toSampleOne;
    private Path toHumanFromSampleOne;
    private Path toSampleTwo;
    private Path toHumanFromSampleTwo;
    private Path toHumanPickFromSpikes;
    private Path toDropZoneTwo;
    private Path toHumanFromDropTwo;
    private Path toDropZoneThree;
    private Path toHumanFromDropThree;
    private Path toDropZoneFour;
    private Path toHumanFromDropFour;
    private Path toDropZoneFive;

    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        toDropZoneOne = new Path(new BezierLine(
                new Point(0, 0, Point.CARTESIAN),
                new Point(30.3, 3.6, Point.CARTESIAN)));
        toDropZoneOne.setConstantHeadingInterpolation(0);

        toSampleOne = new Path(new BezierCurve(
                new Point(30.3, 3.6, Point.CARTESIAN),
                new Point(10, 5.3, Point.CARTESIAN),
                new Point(10, -23, Point.CARTESIAN),
                new Point(10, -24, Point.CARTESIAN),
                new Point(10, -25, Point.CARTESIAN),
                new Point(13, -25, Point.CARTESIAN),
                new Point(15, -25, Point.CARTESIAN),
                new Point(44, -25, Point.CARTESIAN),
                new Point(45, -26, Point.CARTESIAN),
                new Point(46, -27, Point.CARTESIAN),
                new Point(46, -27, Point.CARTESIAN),
                new Point(46, -39, Point.CARTESIAN)
        ));
        toSampleOne.setConstantHeadingInterpolation(0);

        toHumanFromSampleOne = new Path(new BezierLine(
                new Point(46, -39, Point.CARTESIAN),
                new Point(8, -39, Point.CARTESIAN)));
        toHumanFromSampleOne.setConstantHeadingInterpolation(0);

        toSampleTwo = new Path(new BezierCurve(
                new Point(8, -39, Point.CARTESIAN),
                new Point(44, -39, Point.CARTESIAN),
                new Point(46, -39, Point.CARTESIAN),
                new Point(46, -40, Point.CARTESIAN),
                new Point(46, -44, Point.CARTESIAN)
        ));
        toSampleTwo.setConstantHeadingInterpolation(0);

        toHumanFromSampleTwo= new Path(new BezierLine(
                new Point(53,-38,Point.CARTESIAN),
                new Point(10,-38,Point.CARTESIAN)
        )
        );
        toHumanFromSampleTwo.setConstantHeadingInterpolation(Math.toRadians(0));

        toHumanPickFromSpikes= new Path(new BezierCurve(
                new Point(10,-38,Point.CARTESIAN),
                new Point(17,-25,Point.CARTESIAN),
                new Point(4.5,-25,Point.CARTESIAN)
        )
        );
        toHumanPickFromSpikes.setConstantHeadingInterpolation(Math.toRadians(2));

        toDropZoneTwo= new Path(new BezierCurve(
//                new Point(4,-25,Point.CARTESIAN),

//                new Point(11,-25,Point.CARTESIAN),
//                new Point(20,-25,Point.CARTESIAN),
//                new Point(22,-25,Point.CARTESIAN),
//                new Point(22,-20,Point.CARTESIAN),
////                new Point(22,-16,Point.CARTESIAN),
////                new Point(22,-9,Point.CARTESIAN),
//                new Point(22,12,Point.CARTESIAN),
//                new Point(22,16,Point.CARTESIAN),
////                new Point(25,15,Point.CARTESIAN),
//                new Point(31.15,16,Point.CARTESIAN)
                new Point(5,-25, Point.CARTESIAN),
                new Point(28,10,Point.CARTESIAN),
                new Point(30.5,16.75,Point.CARTESIAN)
        )
        );
        toDropZoneTwo.setConstantHeadingInterpolation(0);

        toHumanFromDropTwo= new Path(new BezierCurve(
//                new Point(30,11,Point.CARTESIAN),
//                new Point(25,15.5,Point.CARTESIAN),
//                new Point(17,10.6,Point.CARTESIAN),
//                new Point(11.5,5.78,Point.CARTESIAN),
//                new Point(9.3,3.7,Point.CARTESIAN),
//                new Point(6.9,0.6969,Point.CARTESIAN),
//                new Point(6.9,-25,Point.CARTESIAN),
//                new Point(7,-25,Point.CARTESIAN)
                new Point(31,16.75,Point.CARTESIAN),
                new Point(20,13,Point.CARTESIAN),
//                new Point(10,0,Point.CARTESIAN),
                new Point(10,-25,Point.CARTESIAN),
                new Point(5,-25,Point.CARTESIAN)
        )
        );
        toHumanFromDropTwo.setConstantHeadingInterpolation(Math.toRadians(0));

        toDropZoneThree= new Path(new BezierCurve(
//                new Point(4,-25,Point.CARTESIAN),
//                new Point(11,-25,Point.CARTESIAN),
//                new Point(20,-25,Point.CARTESIAN),
//                new Point(22,-25,Point.CARTESIAN),
//                new Point(22,-20,Point.CARTESIAN),
////                new Point(22,-16,Point.CARTESIAN),
////                new Point(22,-9,Point.CARTESIAN),
//                new Poi33nt(22,12,Point.CARTESIAN),
//                new Point(22,16,Point.CARTESIAN),
////                new Point(25,15,Point.CARTESIAN),
//                new Point(31.15,14.5,Point.CARTESIAN)
                new Point(4.9,-25, Point.CARTESIAN),
                new Point(28,12,Point.CARTESIAN),
                new Point(31,15,Point.CARTESIAN)
        )
        );
        toDropZoneThree.setConstantHeadingInterpolation(0);

        toHumanFromDropThree= new Path(new BezierCurve(
//                new Point(30,11,Point.CARTESIAN),
//                new Point(25,15.5,Point.CARTESIAN),
//                new Point(17,10.6,Point.CARTESIAN),
//                new Point(11.5,5.78,Point.CARTESIAN),
//                new Point(9.3,3.7,Point.CARTESIAN),
//                new Point(6.9,0.6969,Point.CARTESIAN),
//                new Point(6.9,-25,Point.CARTESIAN),
//                new Point(7,-25,Point.CARTESIAN)
                new Point(31,15,Point.CARTESIAN),
                new Point(20,13,Point.CARTESIAN),
//                new Point(10,0,Point.CARTESIAN),
                new Point(10,-25,Point.CARTESIAN),
                new Point(4.5,-25,Point.CARTESIAN)
        )
        );
        toHumanFromDropThree.setConstantHeadingInterpolation(Math.toRadians(0));

        toDropZoneFour= new Path(new BezierCurve(
//                new Point(4,-25,Point.CARTESIAN),
//                new Point(11,-25,Point.CARTESIAN),
//                new Point(20,-25,Point.CARTESIAN),
//                new Point(22,-25,Point.CARTESIAN),
//                new Point(22,-20,Point.CARTESIAN),
////                new Point(22,-16,Point.CARTESIAN),
////                new Point(22,-9,Point.CARTESIAN),
//                new Point(22,12,Point.CARTESIAN),
//                new Point(22,16,Point.CARTESIAN),
////                new Point(25,15,Point.CARTESIAN),
//                new Point(31.15,13,Point.CARTESIAN)
                new Point(4.5,-25, Point.CARTESIAN),
                new Point(28,14,Point.CARTESIAN),
                new Point(30.85,13.5,Point.CARTESIAN)
        )
        );
        toDropZoneFour.setConstantHeadingInterpolation(0);

        toHumanFromDropFour= new Path(new BezierCurve(
//                new Point(30,11,Point.CARTESIAN),
//                new Point(25,15.5,Point.CARTESIAN),
//                new Point(17,10.6,Point.CARTESIAN),
//                new Point(11.5,5.78,Point.CARTESIAN),
//                new Point(9.3,3.7,Point.CARTESIAN),
//                new Point(6.9,0.6969,Point.CARTESIAN),
//                new Point(6.9,-25,Point.CARTESIAN),
//                new Point(7,-25,Point.CARTESIAN)
                new Point(30.85,13.5,Point.CARTESIAN),
                new Point(20,12,Point.CARTESIAN),
//                new Point(10,0,Point.CARTESIAN),
                new Point(10,-25,Point.CARTESIAN),
                new Point(5,-25,Point.CARTESIAN)
        )
        );
        toHumanFromDropFour.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-100));

        toDropZoneFive= new Path(new BezierLine(
                new Point(6.4,-19.95,Point.CARTESIAN),
                new Point(32,14.8,Point.CARTESIAN)
        )
        );
        toDropZoneFive.setConstantHeadingInterpolation(0);

        follower.followPath(toDropZoneOne);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                + " inches forward. The robot will go forward and backward continuously"
                + " along the path. Make sure you have enough room.");
        telemetryA.update();
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();
        if (!follower.isBusy()) {
            if (followPathTwo && !followPathThree && !followPathFour) {
                follower.followPath(toSampleOne);
                followPathTwo = false;
                followPathThree = true;
                followPathFour = false;
            }
            else if (!follower.isBusy()
                    && !followPathTwo
                    && followPathThree
                    && !followPathFour
                    && !followPathFive
                    && !followPathSix
                    && !followPathSeven
                    && !followPathEight
                    && followPathNine
                    && !followPathTen
                    && !followPathEleven
                    && !followPathTwelve
                    && !followPathThirteen
            ) {
                follower.followPath(toHumanFromSampleOne);
                followPathTwo = false;
                followPathThree = false;
                followPathFour = true;
            } else if (!follower.isBusy() && !followPathTwo && !followPathThree && followPathFour) {
                follower.followPath(toSampleTwo);
                followPathTwo = false;
                followPathThree = false;
                followPathFour = false;
            }
//            telemetryA.addLine("going forward");
//            follower.telemetryDebug(telemetryA);
        }
    }
}
