package common.robot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;

/**
 * This is the FollowerConstants class. It holds many constants and parameters for various parts of
 * the Follower. This is here to allow for easier tuning of Pedro Pathing, as well as concentrate
 * everything tunable for the Paths themselves in one place.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/4/2024
 */
@Config
public class robotConstants {

    // Servo Constants
    // armClawServo
    public static double armClawGrabPos = 0.84;
    public static double armClawReleasePos = 0;
    // armWristServo
    public static double armWristCenterPos = 0.42;
    public static double armWristMovementSpeed = 0.01;
    public static double armWristSpecimenDepositPos = 0.98;
    // armPitchServo
    public static double armPitchCenterPos = 0.45;
    public static double armPitchSpecimenGrabPos = 0.34;
    public static double armPitchSampleDepositPos = 0.7;
    public static double armPitchSampleGrabPos = 0.05;
    public static double armPitchSpecimenDepositPos = 0.74;
    public static double armPitchBarrierPos = 0;

    // armPivotServo
    public static double armPivotInitPos = 0.34;
    public static double armPivotSampleGrabPos = 0.18;
    public static double armPivotBarrierPos = 0.29;
    public static double armPivotSpecimenGrabPos = 0.89;
    public static double armPivotSpecimenDepositPos = 0.34;

    // Slide Values
    public static double slideInitPos = 0;
    public static double slideMaxVerLimit = 2050;
    public static double slideMaxHorLimit = 2050;
    public static double slideMovementSpeed = 0;

    // Slide Pivot Values
    public static int slidePivotInitPos = 0;
    public static int slidePivotSampleGrabPos = 0;
    public static int slidePivotSampleDepositPos = 0;
    public static int slidePivotSpecimenGrabPos = 0;
    public static int slidePivotSpecimenDepositPos = 0;
    public static int slidePivotMaxLimit = 2025;

    // Sensor Values

    // PID Controllers
    public static PIDController slidePIDController = new PIDController(0.01,0,0.0001);
    public static PIDController slidePivotPIDController = new PIDController(0.0035,0,0.00003);

    // Feed Forward Constants
    public static double slideFF = 0.003;
    public static double slidePivotFF = 0.0026;

    public static double slideHangFF = -2;
}