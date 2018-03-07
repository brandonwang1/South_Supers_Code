package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.Map;

/* Credit 3491 Fix it */
@TeleOp(name = "servo test", group = "test")
@Disabled
public class TestServo extends BaseTeleop {

    ArrayList<Servo> servos = new ArrayList<Servo>();
    ArrayList<String> names = new ArrayList<String>();
    int testingNum = 0;


    double position = 0;

    @Override
    public void init() {

        for (Map.Entry<String, Servo> servo : hardwareMap.servo.entrySet()) {
            names.add(servo.getKey());
            servos.add(servo.getValue());
        }
        position = servos.get(testingNum).getPosition();
    }

    @Override
    public void loop() {

        telemetry.addData("Switch Servo-x,b |", " Change position-LT, RT");
        telemetry.addData("Changing %s", names.get(testingNum) );
        telemetry.addData("pos: %f", position);


        if (gamepad1.x && runtime.milliseconds() > 100) {
            testingNum--;
            position = servos.get(testingNum).getPosition();
            runtime.reset();
        } else if (gamepad1.b && runtime.milliseconds() > 100) {
            testingNum++;
            position = servos.get(testingNum).getPosition();
            runtime.reset();
        }

        testingNum += 1; //why?
        testingNum -= 1;

        testingNum %= servos.size();

        if (gamepad1.left_trigger > 0.5 && position > 0) {

            position -= 0.01;
            servos.get(testingNum).setPosition(position);

        } else if (gamepad1.right_trigger > 0.5  && position < 1) {

            position += 0.01;
            servos.get(testingNum).setPosition(position);

        }
    }


}
