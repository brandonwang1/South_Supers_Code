package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "System Test", group = "Linear Opmode")
@Disabled
public class TestSystems extends BaseAutonomous {

    @Override
    public void runOpMode() {
        mysteryRobot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Status", "Wait For Start");
        telemetry.update();
        waitForStart();
        // Wait for imu calibration? Not necessary?

        telemetry.addData("PHASE 1-MOTORS", "CHECK TELEMETRY");
        telemetry.update();
        encoderAccDrive(1400, 0.3);
        encoderAccDrive(1500, -0.3);
        insertGamepadBreakpoint();

        insertGamepadBreakpoint();

        insertGamepadBreakpoint();

        insertGamepadBreakpoint();

        telemetry.addData("PHASE 2-SERVOS", "CHECK TELEMETRY");
        telemetry.update();
/*
        mysteryRobot.beaconServo.setPosition(mysteryRobot.beaconRight);
        mysteryRobot.releaseServo.setPosition(mysteryRobot.releaseOpen);
        telemetry.addData("Servos", mysteryRobot.beaconServo.getPosition() + ", "
        );
        robotSleep(2000);

        mysteryRobot.beaconServo.setPosition(mysteryRobot.beaconLeft);
        mysteryRobot.releaseServo.setPosition(mysteryRobot.releaseClosed);
        robotSleep(2000);
*/
        telemetry.addData("PHASE 3-SENSORS", "CHECK TELEMETRY");
        telemetry.update();
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