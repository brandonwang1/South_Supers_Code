package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Judging Session", group = "Competition")
public class JudgingSession extends BaseAutonomous {

    @Override
    public void runOpMode() {
        mysteryRobot.autonomousInit(hardwareMap);
      /*  mysteryRobot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        while (!isStarted()) {
            telemetry.addData("Mode: ", "Judging Session");
            telemetry.update();
        }
        mysteryRobot.glyphAlignmentMotor.setPower(0.4);

        sleep(5000);

        mysteryRobot.topClampServo.setPosition(mysteryRobot.TC_CLOSE);
        sleep(500);
        mysteryRobot.topClampLiftServo.setPosition(mysteryRobot.TC_TOP);
        sleep(5000);


        mysteryRobot.bottomClampServo.setPosition(mysteryRobot.BC_CLOSE);
        sleep(500);
        mysteryRobot.glyphAlignmentMotor.setPower(0);

        insertGamepadBreakpoint();


        mysteryRobot.liftMotor.setPower(0.7);
        sleep(2000);
        mysteryRobot.liftMotor.setPower(0);

        insertGamepadBreakpoint();


        mysteryRobot.glyphFBServo.setPosition(mysteryRobot.FB_FRONT);
        sleep(500);

        insertGamepadBreakpoint();

        mysteryRobot.glyphLRServo.setPosition(mysteryRobot.LR_RIGHT);
        sleep(500);

        insertGamepadBreakpoint();

        mysteryRobot.glyphLRServo.setPosition(mysteryRobot.LR_LEFT);
        sleep(500);

        insertGamepadBreakpoint();

        mysteryRobot.topClampServo.setPosition(mysteryRobot.TC_OPEN);
        mysteryRobot.bottomClampServo.setPosition(mysteryRobot.BC_OPEN);
        sleep(500);

        insertGamepadBreakpoint();

        mysteryRobot.glyphFBServo.setPosition(mysteryRobot.FB_BACK);
        sleep(500);

        insertGamepadBreakpoint();

        mysteryRobot.liftMotor.setPower(-0.7);
        sleep(2000);
        mysteryRobot.liftMotor.setPower(0);

        insertGamepadBreakpoint();

        mysteryRobot.glyphSystemReset();
        sleep(500);
        mysteryRobot.glyphAlignmentMotor.setPower(0.4);
        sleep(5000);
        mysteryRobot.glyphAlignmentMotor.setPower(0);






*/
    }

    @Override
    protected int getDelay() {
        return 0;
    }

    @Override
    protected Boolean getRedAlliance() {
        return false;
    }


}
