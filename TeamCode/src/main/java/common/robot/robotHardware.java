package common.robot;

import androidx.annotation.GuardedBy;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import com.pedropathing.localization.PoseUpdater;
//import com.pedropathing.localization.Localizer;
//import org.firstinspires.ftc.teamcode.common.pathing.localization.FusionLocalizer;
//import org.firstinspires.ftc.teamcode.common.pathing.localization.PoseUpdater;
//import org.firstinspires.ftc.teamcode.common.subsystem.followerSubsystem;
//import org.firstinspires.ftc.teamcode.common.subsystem.intakeSubsystem;
import common.robot.sensors.LimitSwitch;
import common.robot.sensors.Sensors;
//import common.subsystems.armSubsystem;
import common.subsystems.armSubsystem;
import common.subsystems.slideSubsystem;
import common.subsystems.slidePivotSubsystem;
import common.subsystems.slidePivotSubsystem;
//import common.subsystems.slideSubsystem;
import common.util.wrappers.MEActuator;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import common.robot.robotConstants;

public class robotHardware {

    private static robotHardware instance = null;
    private boolean enabled;
    public HardwareMap hardwareMap;

    //    public followerSubsystem follower;
//    public PoseUpdater poseUpdater;
//    public FusionLocalizer localizer;
    public Servo armClawServo, armWristServo, armPitchServo, armPivotServo, ledFlashServo;
    public DcMotorEx slideLeft, slideRight, slidePivotLeft, slidePivotRight;
    public LimitSwitch slidePivotLeftLimit, slidePivotRightLimit, slideLimit;
    public DcMotorEx leftFront, leftRear, rightFront, rightRear;
//    public Encoder strafe, forward;
    public RevColorSensorV3 colorsensor;
    public List<DcMotorEx> driveMotors;
    //    public HashMap<Sensors.SensorType, Object> values;
    public MEActuator slideMotor, slidePivotMotor;

    public ArrayList<Subsystem> subsystems;

    public armSubsystem armSub;
    public slidePivotSubsystem slidePivotSub;
//    public intakeSubsystem intakeSub;
    public Follower follower;
    public slideSubsystem slideSub;
    public List<LynxModule> modules;
    public LynxModule CONTROL_HUB;


    //    public PhotonLynxVoltageSensor battery;

    public VoltageSensor battery;
    private final Object imuLock = new Object();
    @GuardedBy("imuLock")
    public IMU imu;
    private Thread imuThread;
    private double imuAngle = 0;
    private double imuOffset = 0;
    private double startOffset = 0;
//    public Localizer localizer;
    private double startTime;
    public HashMap<Sensors.SensorType, Object> values;
    public Limelight3A limelight;

    public static robotHardware getInstance() {
        if (instance == null) {
            instance = new robotHardware();
        }
        instance.enabled = true;
        return instance;
    }

    public void init(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.values = new HashMap<>();
        battery = hardwareMap.voltageSensor.get("Control Hub");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        // Servos
        armClawServo = hardwareMap.get(Servo.class, "armclawServo");
        armWristServo = hardwareMap.get(Servo.class, "armwristServo");
        armPivotServo = hardwareMap.get(Servo.class, "armpivotServo");
        armPitchServo = hardwareMap.get(Servo.class, "armpitchServo");
        ledFlashServo = hardwareMap.get(Servo.class, "ledflashServo");

        // Drivetrain DcMotors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        // Mech DcMotors
        slideLeft = hardwareMap.get(DcMotorEx.class, "slideLeft");
        slideRight = hardwareMap.get(DcMotorEx.class, "slideRight");
        slidePivotLeft = hardwareMap.get(DcMotorEx.class, "slidepivotLeft");
        slidePivotRight = hardwareMap.get(DcMotorEx.class, "slidepivotRight");

        // Limit Switches
        slidePivotLeftLimit = new LimitSwitch(hardwareMap.get(DigitalChannel.class, "hpivotleftLimit"));
        slidePivotRightLimit = new LimitSwitch(hardwareMap.get(DigitalChannel.class, "hpivotrightLimit"));
        slideLimit = new LimitSwitch(hardwareMap.get(DigitalChannel.class, "slideLimit"));

        //Encoders
//        strafe = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftFront")));
//        forward = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightFront")));

        colorsensor = hardwareMap.get(RevColorSensorV3.class, "armclawcolorSensor");

        slidePivotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePivotRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slidePivotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slidePivotLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightFront.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.REVERSE);
        slideRight.setDirection(DcMotorEx.Direction.REVERSE);

        //Setting Drivetrain Motor Types and Modes
        driveMotors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);
        // Motor Type
        for (DcMotorEx motor : driveMotors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        // Motor Mode
        for (DcMotorEx motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        //Setting Actuators
        slideMotor = new MEActuator(() -> slideRight.getCurrentPosition(), () -> intSubscriber(Sensors.SensorType.SLIDE_LIMIT), slideRight);
        slideMotor.setPIDController(robotConstants.slidePIDController);
        slideMotor.setFeedforward(MEActuator.FeedforwardMode.CONSTANT, robotConstants.slideFF);

        slidePivotMotor = new MEActuator(() -> (slidePivotLeft.getCurrentPosition()+slidePivotRight.getCurrentPosition())/2, () -> intSubscriber(Sensors.SensorType.SLIDE_LIMIT), slidePivotLeft, slidePivotRight);
        slidePivotMotor.setPIDController(robotConstants.slidePivotPIDController);
        slidePivotMotor.setFeedforward(MEActuator.FeedforwardMode.CONSTANT, robotConstants.slidePivotFF);

        armSub = new armSubsystem();
        slideSub = new slideSubsystem();
        slidePivotSub = new slidePivotSubsystem();
//        follower = new followerSubsystem();

    }
        public double doubleSubscriber(Sensors.SensorType topic) {
            Object value = values.getOrDefault(topic, 0.0);
            if (value instanceof Integer) {
                return ((Integer) value).doubleValue();
            } else if (value instanceof Double) {
                return (Double) value;
            } else {
                throw new ClassCastException();
            }
        }
        public int intSubscriber(Sensors.SensorType topic) {
            Object value = values.getOrDefault(topic, 0);
            if (value instanceof Integer) {
                return (Integer) value;
            } else if (value instanceof Double) {
                return ((Double) value).intValue();
            } else {
                throw new ClassCastException();
            }
        }

    public void read(){
        values.put(Sensors.SensorType.SLIDE_LIMIT, slideLimit.isPressed());
        values.put(Sensors.SensorType.SLIDE_LIMIT, (slidePivotLeftLimit.isPressed()|| slidePivotRightLimit.isPressed()));
        values.put(Sensors.SensorType.SLIDE_ENCODER, slideRight.getCurrentPosition());
        values.put(Sensors.SensorType.SLIDE_ENCODER, slidePivotRight.getCurrentPosition());

        slideSub.read();
//        depositSub.read();
//        follower.read();
//        intakeSub.read();
        armSub.read();
        slidePivotSub.read();

    }
    public void periodic(){
//        poseUpdater.periodic();
        armSub.periodic();
//        intakeSub.periodic();
        slideSub.periodic();
        slidePivotSub.periodic();
//        follower.periodic();
//        depositSub.periodic();
    }
    public void write(){
        armSub.write();
//        follower.write();
//        intakeSub.write();
        slideSub.write();
        slidePivotSub.write();
//        depositSub.write();
    }
}


