package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MysteryRobot;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;

@TeleOp(name = "BaseTeleop", group = "Competition")  // @Autonomous(...) is the other common choice
public class BaseTeleop extends OpMode {
    ElapsedTime runtime = new ElapsedTime();
    MysteryRobot mysteryRobot = new MysteryRobot(); // Get MysteryRobot Config.
    GamepadWrapper joy1 = new GamepadWrapper();
    GamepadWrapper joy2 = new GamepadWrapper();

    float relicArmPos = MysteryRobot.RELIC_ARM_INIT;
    int stateFlip = 0;
    float currentRelicPos = MysteryRobot.RELIC_HAND_OPEN;

    boolean bPrevious = false;

    boolean disableClaw;
    @Override
    public void init() {
        mysteryRobot.teleopInit(hardwareMap);
        telemetry.addData("Status", "Initialized");

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void stop() {
    }


    @Override
    public void loop() {

        joy1.update(gamepad1); // Update Toggles
        joy2.update(gamepad2);

        driveControl();
        glyphControl();
        relicControl();

        telemetry.addData("Throttle(L,R)", mysteryRobot.leftBackMotor.getPower() + ", " + mysteryRobot.rightBackMotor.getPower());
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("RelicMotor", mysteryRobot.relicArmMotor.getCurrentPosition());
        telemetry.addData("RelicArmPos", relicArmPos);
    }


    private void driveControl() {

        if(joy1.toggle.x || joy2.toggle.x){
            mysteryRobot.setLeftRightPower(joy1.scaleJoystick(gamepad1.right_stick_y, -0.5),
                    joy1.scaleJoystick(gamepad1.left_stick_y, -0.5));
            if(Math.abs(gamepad1.right_stick_y)<0.05&& Math.abs(gamepad1.left_stick_y)<0.05){
                mysteryRobot.setLeftRightPower(joy2.scaleJoystick(gamepad2.right_stick_y, -0.5),
                        joy2.scaleJoystick(gamepad2.left_stick_y, -0.5));
            }

            telemetry.addData("Slow Mode", "ON");

        }else{
            mysteryRobot.setLeftRightPower(joy1.scaleJoystick(gamepad1.right_stick_y, -1),
                    joy1.scaleJoystick(gamepad1.left_stick_y, -1));
            if(Math.abs(gamepad1.right_stick_y)<0.05&& Math.abs(gamepad1.left_stick_y)<0.05){
            mysteryRobot.setLeftRightPower(joy2.scaleJoystick(gamepad2.right_stick_y, -1),
                    joy2.scaleJoystick(gamepad2.left_stick_y, -1));
                }
            telemetry.addData("Slow Mode", "OFF");

        }
    }

    private void glyphControl() {

        if (joy1.toggle.left_bumper||joy2.toggle.left_bumper) {
            mysteryRobot.glyphLeftMotor.setPower(-1);
            mysteryRobot.glyphRightMotor.setPower(-1);
            stateFlip++;
            if(stateFlip>20) {
                mysteryRobot.glyphFlipRightServo.setPosition(MysteryRobot.GLYPH_FLIP_UP);
                stateFlip = 0;
            }

            joy1.toggle.a = false;
            joy2.toggle.a = false;
        }else if(joy1.toggle.a||joy2.toggle.a){
            mysteryRobot.glyphLeftMotor.setPower(-1);
            mysteryRobot.glyphRightMotor.setPower(-1);
            stateFlip++;
            if(stateFlip>20) {
                mysteryRobot.glyphFlipRightServo.setPosition(MysteryRobot.GLYPH_FLIP_FLAT);
                stateFlip = 0;
            }
        }else {
            mysteryRobot.glyphFlipRightServo.setPosition(MysteryRobot.GLYPH_FLIP_DOWN);
            mysteryRobot.glyphLeftMotor.setPower(0);
            mysteryRobot.glyphRightMotor.setPower(0);
        }


        if (joy1.toggle.right_bumper||joy2.toggle.right_bumper) {
            mysteryRobot.glyphLeftMotor.setPower(-1);
            mysteryRobot.glyphRightMotor.setPower(-1);
        }else {
            mysteryRobot.glyphLeftMotor.setPower(0);
            mysteryRobot.glyphRightMotor.setPower(0);
        }
        if ((gamepad1.right_trigger>0.5f)||(gamepad2.right_trigger>0.5f)) {
            mysteryRobot.glyphLeftMotor.setPower(1);
            mysteryRobot.glyphRightMotor.setPower(1);
        }
    }

    private void relicControl() {

        if (gamepad1.dpad_up || gamepad2.dpad_up)
            mysteryRobot.relicArmMotor.setPower(MysteryRobot.RELIC_ARM_OUT);
        else
            mysteryRobot.relicArmMotor.setPower(0);

        if (gamepad1.dpad_down || gamepad2.dpad_down)
            mysteryRobot.relicArmMotor.setPower(MysteryRobot.RELIC_ARM_IN);
        else
            mysteryRobot.relicArmMotor.setPower(0);

        if (gamepad1.dpad_right || gamepad2.dpad_right) {
            relicArmPos -= MysteryRobot.RELIC_ARM_INC;
            if (relicArmPos < 0)
                relicArmPos = 0.0f;

            mysteryRobot.relicArmServo.setPosition(relicArmPos);
        }

        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            relicArmPos += MysteryRobot.RELIC_ARM_INC;
            if (relicArmPos > 1)
                relicArmPos = 1.0f;

            mysteryRobot.relicArmServo.setPosition(relicArmPos);
        }


        /*if(joy1.toggle.y ||joy2.toggle.y) {
            mysteryRobot.relicHandServo.setPosition(0.55);
        } else if (joy1.toggle.b || joy2.toggle.b) {
            mysteryRobot.relicHandServo.setPosition(MysteryRobot.RELIC_HAND_CLOSED);

        } else {
            mysteryRobot.relicHandServo.setPosition(MysteryRobot.RELIC_HAND_OPEN);
        }*/

        if (joy1.toggle.b||joy2.toggle.b) {
            mysteryRobot.relicHandServo.setPosition(MysteryRobot.RELIC_HAND_CLOSED);

            joy1.toggle.y = false;
            joy2.toggle.y = false;
        }else if(joy1.toggle.y||joy2.toggle.y){
            mysteryRobot.relicHandServo.setPosition(MysteryRobot.RELIC_HAND_READY);
        }else {
            mysteryRobot.relicHandServo.setPosition(MysteryRobot.RELIC_HAND_OPEN);

        }



    }

}
