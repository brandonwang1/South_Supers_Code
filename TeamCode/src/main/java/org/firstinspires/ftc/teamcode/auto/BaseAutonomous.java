package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.MysteryRobot;

import static java.lang.String.format;


/**
 * An abstract class that is the basis of the mysteryRobot's autonomous routines. This
 * is implemented in auto.modes (which are routines that do actions).
 */

public abstract class BaseAutonomous extends LinearOpMode {

    //omniRobot mysteryRobot = new omniRobot(); // Get MysteryRobot Config.
    MysteryRobot mysteryRobot = new MysteryRobot(); // Get MysteryRobot Config.
    private ElapsedTime timeoutTimer = new ElapsedTime(); // Used as a watchdog to kill anything that runs for too long
    RelicRecoveryVuMark detectedVuMark = RelicRecoveryVuMark.UNKNOWN;

    /* Required settings for each autonomous mode */
    abstract protected int getDelay(); // Delay in ms
    abstract protected Boolean getRedAlliance(); // Select alliance

    @Override
    public void runOpMode() {
        mysteryRobot.autonomousInit(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        while (!isStarted()) {
            telemetry.addData("Status", getRedAlliance() ? "RED alliance ready" : "BLUE alliance ready");
            telemetry.addData("Glyph box: ", mysteryRobot.keyFinder.getVisibleVuMark());
            telemetry.addData("DELAY(ms):", Integer.toString(getDelay()));
            telemetry.addData("Color: ", "Red " + mysteryRobot.colorSensor.red() +
                    ",Blue " + mysteryRobot.colorSensor.blue() ); // Color Sensor
            telemetry.addData("Voltage: ", mysteryRobot.voltageSensor.getVoltage());
            telemetry.update();
        }

        telemetry.addData("Status", "In delay");
        telemetry.update();
        sleep(getDelay());

        //removeJewel();

    }
    // Grabs the glyph(s)
    void grabGlyph() {

        timeoutTimer.reset();
        mysteryRobot.glyphLeftMotor.setPower(-1);
        mysteryRobot.glyphRightMotor.setPower(-1);
        sleep(1000);
        mysteryRobot.glyphLeftMotor.setPower(0);
        mysteryRobot.glyphRightMotor.setPower(0);


    }

    // Spits out the glyph
    void placeGlyph() {
        timeoutTimer.reset();
        mysteryRobot.glyphLeftMotor.setPower(1);
        mysteryRobot.glyphRightMotor.setPower(1);
        sleep(200);
        mysteryRobot.glyphInOut.setPosition(0.65);

        sleep(500);
        mysteryRobot.glyphLeftMotor.setPower(0.0);
        mysteryRobot.glyphRightMotor.setPower(0.0);

        sleep(100);
        encoderAccDrive(20, -0.3);
        sleep(300);
        encoderAccDrive(60, .3);
        sleep(100);
        mysteryRobot.glyphLeftMotor.setPower(0.6);
        mysteryRobot.glyphRightMotor.setPower(0.6);
        encoderAccDrive(40, -.3);
    }
    //a


    public void timeDrive(double milliseconds, double power) {
        mysteryRobot.setRunUsingEncoderMode();
        timeoutTimer.reset();
        while (timeoutTimer.milliseconds() < milliseconds && opModeIsActive()) {
            mysteryRobot.setLeftRightPower(power, power);
        }
        mysteryRobot.stopRobot();
    }

     /* TODO: Add Trapezoidal acceleration, voltage compensation */
    public void encoderDrive(double distance, double power) {
        mysteryRobot.setRunUsingEncoderMode();
        int startDistance = mysteryRobot.getEncoderPosition();
        timeoutTimer.reset();
        while (Math.abs(mysteryRobot.getEncoderPosition() - startDistance) < Math.abs(distance) &&
                timeoutTimer.milliseconds() < Math.abs(distance) /  Math.abs(power) * 5 && opModeIsActive()) {
            mysteryRobot.setLeftRightPower(power, power);
            telemetry.addData("Position", mysteryRobot.getEncoderPosition());
            telemetry.update();
        }
        mysteryRobot.stopRobot();
    }

    /** drives with encoder but adds trapezoidal acceleration
        @param distance in encoder tics, distance the mysteryRobot should move (always positive)
        @param power between -1 and 1, depending on direction
     **/
    void encoderAccDrive(double distance, double power) {
        mysteryRobot.setRunWithoutEncoderMode();
        int startDistance = mysteryRobot.getEncoderPosition();
        timeoutTimer.reset();
        double distanceTraveled = 0 ;
        double set_p;
        double acc_point = Math.min(30,distance/2);
        while (distanceTraveled < Math.abs(distance) && timeoutTimer.milliseconds() <
                Math.abs(distance) / Math.abs(power) * 5 && opModeIsActive()) {
            distanceTraveled = Math.abs(mysteryRobot.getEncoderPosition() - startDistance);
            if(distanceTraveled > distance){ // We are done here
                break;
            } else if (distanceTraveled < acc_point) {            // Accelerate at the start
                set_p = (0.3+ Math.sqrt(2 * distanceTraveled * Constants.kMaxAcceleration)) * Math.signum(power);
            } else if (distance - distanceTraveled < acc_point) { // Decelerate at the end
                set_p = Math.sqrt(2 * (distance - distanceTraveled) * Constants.kMaxAcceleration) * Math.signum(power);
            } else {
                set_p = power;
            }
            set_p  = set_p > power ? power : set_p; // Make sure we don't go too fast
            mysteryRobot.setLeftRightPower(-1.*set_p, -1.*set_p);
            Log.d("RobotDebug_Position", String.valueOf(distanceTraveled));
            Log.d("RobotDebug_Power", String.valueOf(set_p));
        }
        Log.d("RobotDebug_Time", String.valueOf(timeoutTimer.milliseconds()));
        mysteryRobot.stopRobot();
    }

    /*void encoderGyroDrive(double distance, double power) {
        // TODO: Make a debug method
        mysteryRobot.setRunUsingEncoderMode();
        int startDistance = mysteryRobot.getEncoderPosition();
        float startHeading = mysteryRobot.getHeading();
        timeoutTimer.reset();
        while (Math.abs(mysteryRobot.getEncoderPosition() - startDistance) < Math.abs(distance) &&
                timeoutTimer.milliseconds() < Math.abs(distance) /  Math.abs(power) * 5 && opModeIsActive()) {
            double error_degrees = mysteryRobot.getHeading() - startHeading;
            double correction = mysteryRobot.gyroDriveController.findCorrection(error_degrees); //Get Correction
            correction = Range.clip(correction, -0.3, 0.3); //Limit Correction
            //Log.i("DEBUG_Error", String.valueOf(correction));
            mysteryRobot.setLeftRightPower(power, power);
        }
        mysteryRobot.stopRobot();
    }*/

    /* TODO: Turns using the method set out by
     * https://github.com/Team254/FRC-2017-Public/blob/ab2b7a0c40a76d64e4be4f1c0471d8f2b3a4bcd4/src/com/team254/frc2017/subsystems/Drive.java#L479
     * https://www.chiefdelphi.com/forums/showpost.php?p=1686077&postcount=7
      * Make sure power also = 0 */
    /*void encoderGyroTurn(double deg, double leftMultiplier, double rightMultiplier) {
        double target_angle = mysteryRobot.getHeading() - deg;
        int lStartDistance = mysteryRobot.leftBackMotor.getCurrentPosition();
        int rStartDistance = mysteryRobot.rightBackMotor.getCurrentPosition();
        timeoutTimer.reset();
        while (Math.abs(target_angle - mysteryRobot.getHeading()) % 360 > 4 &&
                timeoutTimer.milliseconds() < 3000 && opModeIsActive()) {
            double error_degrees = (target_angle - mysteryRobot.getHeading()) % 360; //Compute Error in degrees
            double lWheelError =  (lStartDistance - mysteryRobot.leftBackMotor.getCurrentPosition()) - error_degrees * 2.65;
            double rWheelError = error_degrees * 2.65 + (rStartDistance - mysteryRobot.rightBackMotor.getCurrentPosition());
            Log.d("RobotDebug_Err", format("DegErr %f , LErr %f, RErr %f", error_degrees, lWheelError, rWheelError));
            double l_power = Range.clip(mysteryRobot.gyroTurnController.findCorrection(lWheelError), -0.5,0.5);
            double r_power = Range.clip(mysteryRobot.gyroTurnController.findCorrection(rWheelError), -0.5,0.5);
            Log.d("RobotDebug_TPower", format("L %f, R %f", l_power, r_power));
            mysteryRobot.setLeftRightPower(l_power, r_power);
        }
        mysteryRobot.setLeftRightPower(0,0);
        Log.d("RobotDebug_DONE", format("DONE %f", deg));

    }*/

    void gyroTurn(double deg, double leftMultiplier, double rightMultiplier) {
        double target_angle = mysteryRobot.getHeading() - deg;
        timeoutTimer.reset();
        while (Math.abs((target_angle - mysteryRobot.getHeading())% 360) > 1 &&
                timeoutTimer.milliseconds() < 3000 && opModeIsActive()) {
            double error_degrees = (target_angle - mysteryRobot.getHeading()) % 360; //Compute Error
            double motor_output = Range.clip(mysteryRobot.gyroTurnController.findCorrection(error_degrees), -1,1); //Get Correction
           if (Math.abs(motor_output) < .65) {
               motor_output = .65 * Math.signum(motor_output);
           }
           if (leftMultiplier+rightMultiplier < 1.1) {
               motor_output *=2;
           }
            Log.d("RobotDebug_Err", format("DegErr %f , Motor %f", error_degrees, motor_output));
            mysteryRobot.setLeftRightPower(-1*motor_output, motor_output);
        }
        mysteryRobot.setLeftRightPower(0,0);
    }

    /*void basicTurn(double deg) { // Omni bot is 2.65 enc/degree
        mysteryRobot.setRunWithoutEncoderMode();
        double target_angle = mysteryRobot.getHeading() - deg;
        timeoutTimer.reset();
        int leftfPosition = mysteryRobot.leftFrontMotor.getCurrentPosition();
        int rightfPosition = mysteryRobot.rightFrontMotor.getCurrentPosition();
        int rightbPosition = mysteryRobot.rightBackMotor.getCurrentPosition();
        int leftbPosition = mysteryRobot.leftBackMotor.getCurrentPosition();
        Log.d("RobotDebug_StartHeading", String.valueOf(mysteryRobot.getHeading()));
        while (Math.abs(target_angle - mysteryRobot.getHeading()) % 360 > 4 &&
                timeoutTimer.milliseconds() < 3000 && opModeIsActive()) {
            //Log.d("RobotDebug_Heading", String.valueOf(mysteryRobot.getHeading()));
            //Log.d("RobotDebug_HeadingLeft", String.valueOf(Math.abs(target_angle - mysteryRobot.getHeading())));
            mysteryRobot.setLeftRightPower(0.5*Math.signum(deg), -0.5*Math.signum(deg));
        }
        Log.d("RobotDebug_LFDif", String.valueOf(mysteryRobot.leftFrontMotor.getCurrentPosition() - leftfPosition));
        Log.d("RobotDebug_LBDif", String.valueOf(mysteryRobot.leftBackMotor.getCurrentPosition() - leftbPosition));
        Log.d("RobotDebug_RFDif", String.valueOf(mysteryRobot.rightFrontMotor.getCurrentPosition() - rightfPosition));
        Log.d("RobotDebug_RBDif", String.valueOf(mysteryRobot.rightBackMotor.getCurrentPosition() - rightbPosition));
        Log.d("RobotDebug_EndHeading", String.valueOf(mysteryRobot.getHeading()));

        mysteryRobot.setLeftRightPower(0,0);
    }*/

    void gyroTurn(double deg) {
        gyroTurn(deg,1,1);
    }

    private void removeJewel() {
        mysteryRobot.topJewelServo.setPosition(0.2);
        sleep(200);
        mysteryRobot.JPUDServo.setPosition(0.93);
        sleep(200);

        mysteryRobot.JPIOServo.setPosition(0.77);
        sleep(200);
        mysteryRobot.JPUDServo.setPosition(0.5);
        sleep(200);

        mysteryRobot.bottomJewelServo.setPosition(0.3);
        sleep(200);
        mysteryRobot.topJewelServo.setPosition(0.37);
        sleep(200);
        mysteryRobot.bottomJewelServo.setPosition(0.17);
        sleep(200);

        if (mysteryRobot.keyFinder.getVisibleVuMark() != RelicRecoveryVuMark.UNKNOWN) {
            detectedVuMark = mysteryRobot.keyFinder.getVisibleVuMark();
        }
        mysteryRobot.keyFinder.closeVuforia();

        telemetry.addData("Vumark Detected %s", detectedVuMark);
        telemetry.update();

        int redTotal = 0;
            int blueTotal = 0;
            for (int i = 0; i < 5; i++) { // JERRY DONT TOUCH THIS
                if (redTotal != 255 && blueTotal != 255) {
                    redTotal += mysteryRobot.colorSensor.red(); // Add to the values
                    blueTotal += mysteryRobot.colorSensor.blue();
                }
                telemetry.addData("Blue, Red", blueTotal + "," + redTotal+ "," + i);
                telemetry.update();
            }
        if (redTotal + blueTotal > 10) { //Only run if with readings
                if ((redTotal < blueTotal) ^ getRedAlliance()) { //XOR blue
                    mysteryRobot.topJewelServo.setPosition(0.25); //hits the ball
                    sleep(200);
                } else if (redTotal != blueTotal) {
                    mysteryRobot.topJewelServo.setPosition(0.49); //hits the ball
                    sleep(200);
                }

        }

        mysteryRobot.bottomJewelServo.setPosition(0.9);//good
        sleep(200);
        mysteryRobot.JPUDServo.setPosition(1);//good
        sleep(500);

        mysteryRobot.JPIOServo.setPosition(0);
        sleep(200);

        mysteryRobot.topJewelServo.setPosition(0);
        sleep(200);

        mysteryRobot.JPUDServo.setPosition(0.93);
        sleep(200);



    } //random edit

    /* Misc useful methods */
    public void insertGamepadBreakpoint() {
        while (opModeIsActive()) {
            telemetry.addData("Hit B to", " continue");
            telemetry.update();
            idle();
            if (gamepad1.b) {
                break;
            }
        }
    }


}
