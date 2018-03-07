package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MysteryRobot;
import org.firstinspires.ftc.teamcode.util.GamepadWrapper;

@TeleOp(name = "PitCrew", group = "Competition")  // @Autonomous(...) is the other common choice
@Disabled
public class PitCrew extends OpMode {
    ElapsedTime runtime = new ElapsedTime();
    //omniRobot mysteryRobot = new omniRobot(); // Get MysteryRobot Config.
    MysteryRobot mysteryRobot = new MysteryRobot(); // Get MysteryRobot Config.
    GamepadWrapper joy1 = new GamepadWrapper();
    GamepadWrapper joy2 = new GamepadWrapper();
    double encoderReset, liftReset, liftPosition, clawReset, clawPosition, rotationReset, rotationPosition = 0;
    boolean isClosed;

    @Override
    public void init() {
        mysteryRobot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        mysteryRobot.setRunWithoutEncoderMode();
        //mysteryRobot.claw.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
        //clawReset = mysteryRobot.claw.getCurrentPosition();


    }

    @Override
    public void loop() {

        joy1.update(gamepad1); // Update Toggles
        joy2.update(gamepad2);

        buttonControl();
        driveControl();

        telemetry.addData("Throttle(L,R)", mysteryRobot.leftBackMotor.getPower() + ", " + mysteryRobot.rightBackMotor.getPower());
        telemetry.addData("Encoders(L,R)", mysteryRobot.leftBackMotor.getCurrentPosition() + ", " + mysteryRobot.rightBackMotor.getCurrentPosition());
        telemetry.addData("Color b/r", mysteryRobot.colorSensor.blue() + "," + mysteryRobot.colorSensor.red());
        telemetry.addData("Status", "Running: " + runtime.toString());
        telemetry.addData("Controls", "x-slow, a,b=claw, lt,rt=lift");
        telemetry.addData("linear", liftPosition);
        telemetry.addData("claw", clawPosition);
        telemetry.addData("rotation", rotationPosition);

    }

    //a
    private void driveControl() {
        //A toggles slow mode
        //If joystick 2's right bumper is toggled, it disables joystick 1's driving
        if (joy1.toggle.x || joy2.toggle.x) {

            mysteryRobot.setLeftRightPower(joy1.scaleJoystick(gamepad1.left_stick_y, .5),
                    joy1.scaleJoystick(gamepad1.right_stick_y, .5));
            telemetry.addData("Slow Mode", "on");

        } else {

            mysteryRobot.setLeftRightPower(joy1.scaleJoystick(gamepad1.left_stick_y, 1),
                    joy1.scaleJoystick(gamepad1.right_stick_y, 1));
            telemetry.addData("Slow Mode", "off");

        }
    }


    private void buttonControl() {
        // Bottom set of claws
        /*if (joy1.toggle.left_bumper || joy2.toggle.left_bumper) {
            mysteryRobot.bLeftClawServo.setPosition(Constants.kBotLeftClawClosed);
            mysteryRobot.bRightClawServo.setPosition(Constants.kBotRightClawClosed);
        } else {
            mysteryRobot.bLeftClawServo.setPosition(Constants.kBotLeftClawOpen);
            mysteryRobot.bRightClawServo.setPosition(Constants.kBotRightClawOpen);
        } */

        /* Claw
            if (gamepad1.left_bumper|| gamepad2.left_bumper) {
                mysteryRobot.claw.setPower(-0.3);//close
            } else  if (gamepad1.right_bumper|| gamepad2.right_bumper) {
                mysteryRobot.claw.setPower(0.3);//open
            } else
                mysteryRobot.claw.setPower(0);
            clawPosition = mysteryRobot.claw.getCurrentPosition()-clawReset;
*/





    }
}






