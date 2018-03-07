package org.firstinspires.ftc.teamcode.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "DT Test", group = "Test")
public class TestDrivetrain extends BaseAutonomous {


    @Override
    public void runOpMode(){
        mysteryRobot.init(hardwareMap);
        waitForStart();
        int distance = mysteryRobot.leftFrontMotor.getCurrentPosition();
        float heading = mysteryRobot.getHeading();

    while (opModeIsActive()){
        if (gamepad1.a) {
            gyroTurn(20);
        } else if (gamepad1.b) {
            gyroTurn(-20);
        } else if (gamepad1.left_bumper) {
            gyroTurn(180);
        } else if (gamepad1.right_bumper) {
            gyroTurn(-180);
        } else if (gamepad1.x) {
            encoderAccDrive(40, 0.4);
        } else if (gamepad1.y) {
            encoderAccDrive(40, -0.4);
        }

        telemetry.addData("heading", mysteryRobot.getHeading()-heading);
        telemetry.addData("distance", mysteryRobot.leftFrontMotor.getCurrentPosition()-distance);
        telemetry.update();
    }

    }

    @Override
    protected int getDelay() {
        return 0;
    }

    @Override
    protected Boolean getRedAlliance() {
        return true;
    }
}
