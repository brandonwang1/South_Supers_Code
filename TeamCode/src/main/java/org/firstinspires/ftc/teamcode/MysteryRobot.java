package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.firstinspires.ftc.teamcode.vision.VuforiaTracker;


/**
 * This class defines the hardware for the robot and its initialization
 */

public class MysteryRobot {

    /* Public OpMode members. */
    public DcMotorEx leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor = null;
    public Servo bottomJewelServo, topJewelServo, JPIOServo, JPUDServo = null;

    public DcMotorEx relicArmMotor = null;
    public DcMotorEx glyphRightMotor = null;
    public DcMotorEx glyphLeftMotor = null;

    public Servo relicArmServo, relicHandServo, glyphFlipRightServo, glyphInOut = null;

    public BNO055IMU imu;
    public VoltageSensor voltageSensor = null;
    public ColorSensor colorSensor = null;

    public PIDController gyroDriveController = new PIDController("Drive", 0.03, 0.0, 0, 0.8),
            gyroTurnController = new PIDController("Turn", 0.0064, 0.0022, 0.0, 0.8);
    public VuforiaTracker keyFinder = new VuforiaTracker();  // Use Tracking library

    public static final float GLYPH_FLIP_UP = 0.55f;
    public static final float GLYPH_FLIP_FLAT = 0.2f;
    public static final float GLYPH_FLIP_DOWN = 0f;

    public static final float RELIC_ARM_INIT = 0.52f;
    public static final float RELIC_ARM_INC = 0.003f;

    public static final float RELIC_ARM_OUT = 1.0f;
    public static final float RELIC_ARM_IN = -1.0f;

    public static final float RELIC_HAND_OPEN = 0.6f;
    public static final float RELIC_HAND_CLOSED = 0.2f;
    public static final float RELIC_HAND_READY = 0.3f;
    /* local OpMode members. */
    HardwareMap hwMap = null;

    /* Constructor */
    public MysteryRobot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor = hwMap.get(DcMotorEx.class, "CFLWM");
        leftBackMotor = hwMap.get(DcMotorEx.class, "CRLWM");
        rightFrontMotor = hwMap.get(DcMotorEx.class,"CFRWM");
        rightBackMotor = hwMap.get(DcMotorEx.class, "CRRWM");

        relicArmMotor = hwMap.get(DcMotorEx.class, "relicarmMotor");
        glyphRightMotor = hwMap.get(DcMotorEx.class, "glyphRightMotor");
        glyphLeftMotor = hwMap.get(DcMotorEx.class, "glyphLeftMotor");

        // Set all motors direction
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        rightBackMotor.setDirection(DcMotor.Direction.FORWARD);

        relicArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        glyphRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightBackMotor.setPower(0);

        relicArmMotor.setPower(0);
        glyphRightMotor.setPower(0);
        glyphLeftMotor.setPower(0);

        // Set all motors to run with/without encoders.
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        relicArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        relicArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glyphRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glyphLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Set Brake Behavior
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        relicArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        JPIOServo = hwMap.get(Servo.class, "JPIOS");
        JPUDServo = hwMap.get(Servo.class, "JPUDS");
        bottomJewelServo = hwMap.get(Servo.class, "JLAS");
        topJewelServo = hwMap.get(Servo.class, "JUAS");

        glyphFlipRightServo = hwMap.get(Servo.class, "glyphFlipRightServo");
        glyphInOut = hwMap.get(Servo.class, "glyphInOut");
        relicArmServo = hwMap.get(Servo.class, "relicArmServo");
        relicHandServo = hwMap.get(Servo.class, "relicClawServo");

        JewelPhoneInit();

       // Initialize Sensors
        imu = ahwMap.get(BNO055IMU.class, "IMU");
        imuInit();
        colorSensor = ahwMap.get(ColorSensor.class, "JCSR");
        colorSensor.enableLed(true);
        voltageSensor = ahwMap.get(VoltageSensor.class, "Expansion Hub 1");
    }

    public void JewelPhoneInit() {
        JPIOServo.setPosition(0);//subtracted .1
        JPUDServo.setPosition(.9);//added .1
        bottomJewelServo.setPosition(0.9);
        topJewelServo.setPosition(0);
    }

    /* Init for Auto*/
    public void autonomousInit(HardwareMap ahwMap) {
        init(ahwMap);
        keyFinder.initVuforia(ahwMap);
        glyphInOut.setPosition(0);

    }
    /*For BaseTeleop Specific Init Code */
    public void teleopInit(HardwareMap ahwMap) {
        init(ahwMap);
        relicHandServo.setPosition(.4);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        relicArmServo.setPosition(RELIC_ARM_INIT);


    }

    /* Initializes IMU */
    public void imuInit() {
        // Set up the parameters with which we will use our IMU.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; // Remove?
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false; // Remove?
        parameters.loggingTag = "IMU"; // Remove?
        imu.initialize(parameters);
    }

    public float getHeading() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public int getEncoderPosition() {
        return (leftBackMotor.getCurrentPosition() + rightBackMotor.getCurrentPosition()) / 2;
    }

    public double getPower() {
        return (leftFrontMotor.getPower() + rightFrontMotor.getPower())/2;
    }

    public void stopRobot() {
        setLeftRightPower(0,0);
    }

    /* sets the power of the left and right motors after clipping to (-1,1)
     */
    public void setLeftRightPower(double leftPower, double rightPower) {
        leftFrontMotor.setPower(Range.clip(leftPower,-1,1));
        leftBackMotor.setPower(Range.clip(leftPower,-1,1));
        rightFrontMotor.setPower(Range.clip(rightPower,-1,1));
        rightBackMotor.setPower(Range.clip(rightPower,-1,1));
    }

    public void setVelocity(double left_degrees_per_sec, double right_degrees_per_sec){
        final double max_desired = Math.max(Math.abs(left_degrees_per_sec), Math.abs(right_degrees_per_sec));
        final double scale = max_desired > Constants.kDriveMaxDps ? Constants.kDriveMaxDps / max_desired : 1.0;
        rightFrontMotor.setVelocity(left_degrees_per_sec * scale, AngleUnit.DEGREES);
        rightBackMotor.setVelocity(right_degrees_per_sec * scale, AngleUnit.DEGREES);
    }

    public void resetEncoders() {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setRunUsingEncoderMode(){
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setRunWithoutEncoderMode(){
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}

