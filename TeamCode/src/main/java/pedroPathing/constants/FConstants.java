package pedroPathing.constants;

import com.pedropathing.localization.Localizers;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class FConstants {
    static {
        FollowerConstants.localizers = Localizers.TWO_WHEEL;

        FollowerConstants.leftFrontMotorName = "leftFront";
        FollowerConstants.leftRearMotorName = "leftRear";
        FollowerConstants.rightFrontMotorName = "rightFront";
        FollowerConstants.rightRearMotorName = "rightRear";

        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

        FollowerConstants.mass = 12.68;

//        FollowerConstants.xMovement = 81.5388590781502;
//        FollowerConstants.yMovement = 65.30124266644745;
//
//        FollowerConstants.forwardZeroPowerAcceleration =   -23.287148582638856;
//        FollowerConstants.lateralZeroPowerAcceleration = -59.21241507081988;
        FollowerConstants.xMovement = 78.39317488924588;
        FollowerConstants.yMovement = 51.59883879210986;

        FollowerConstants.forwardZeroPowerAcceleration = -35.82569993587527;
        FollowerConstants.lateralZeroPowerAcceleration = -73.20368342943081;

        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.14,0,0.044,0);
//        FollowerConstants.translationalPIDFCoefficients.setCoefficients(0.3,0,0.001,0);
        FollowerConstants.useSecondaryTranslationalPID = true;
        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.13,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

//        FollowerConstants.secondaryTranslationalPIDFCoefficients.setCoefficients(0.08,0,0.01,0); // Not being used, @see useSecondaryTranslationalPID

//        FollowerConstants.headingPIDFCoefficients.setCoefficients(1.3,0,0.09,0);
        FollowerConstants.headingPIDFCoefficients.setCoefficients(3.4,0,0.28,0);


        FollowerConstants.useSecondaryHeadingPID = true;
//        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(1,0,0.05,0); // Not being used, @see useSecondaryHeadingPID
        FollowerConstants.secondaryHeadingPIDFCoefficients.setCoefficients(1.2,0,0.1,0); // Not being used, @see useSecondaryHeadingPID

        FollowerConstants.drivePIDFCoefficients.setCoefficients(0.018,0,0.0002,0.2,0);
        FollowerConstants.useSecondaryDrivePID = true;
        FollowerConstants.secondaryDrivePIDFCoefficients.setCoefficients(0.01,0,0.0,0.0001,0); // Not being used, @see useSecondaryDrivePID

        FollowerConstants.zeroPowerAccelerationMultiplier = 4.2;
        FollowerConstants.centripetalScaling = 0.0003;

        FollowerConstants.pathEndTimeoutConstraint = 500;
        FollowerConstants.pathEndTValueConstraint = 0.995;
        FollowerConstants.pathEndVelocityConstraint = 0.1;
        FollowerConstants.pathEndTranslationalConstraint = 0.1;
        FollowerConstants.pathEndHeadingConstraint = 0.007;
    }
}
