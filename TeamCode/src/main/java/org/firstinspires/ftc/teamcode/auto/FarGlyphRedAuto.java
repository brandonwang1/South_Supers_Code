package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MysteryRobot;

@Autonomous(name = "FAR GLYPH Red", group = "Competition")

public class FarGlyphRedAuto extends BaseAutonomous {

    @Override
    public void runOpMode() {
        super.runOpMode();
        encoderAccDrive(75,-0.3);
        sleep(250);




       switch (detectedVuMark) {
            case UNKNOWN: //Should throw an error but just does left for now.
            case LEFT:
                gyroTurn(90);
                sleep(250);
                encoderAccDrive(60,0.3);
                sleep(250);
                gyroTurn(60);
                sleep(250);
                encoderAccDrive(15,0.3);
                sleep(250);

                placeGlyph();

                /*mysteryRobot.glyphLeftMotor.setPower(-1);
                mysteryRobot.glyphRightMotor.setPower(-1);

                gyroTurn(-65);

                encoderAccDrive(30,1);

                gyroTurn(-90);
                encoderAccDrive(50,1);
                encoderAccDrive(10,-1);
                gyroTurn(-30);
                encoderAccDrive(30,1);
                encoderAccDrive(30,-1);
                gyroTurn(30);

                mysteryRobot.glyphLeftMotor.setPower(0);
                mysteryRobot.glyphRightMotor.setPower(0);
                encoderAccDrive(40,-1);

                gyroTurn(90);
                encoderAccDrive(20,1);
                gyroTurn(-90);
                encoderAccDrive(10,1);

                mysteryRobot.glyphFlipRightServo.setPosition(MysteryRobot.GLYPH_FLIP_UP);
                sleep(3000);
                mysteryRobot.glyphFlipRightServo.setPosition(MysteryRobot.GLYPH_FLIP_DOWN);*/
                break;
            case CENTER:
                gyroTurn(90);
                sleep(250);
                encoderAccDrive(31,0.3);
                sleep(250);
                gyroTurn(60);
                sleep(250);
                encoderAccDrive(15,0.3);
                sleep(250);
                placeGlyph();
                break;
            case RIGHT:
                gyroTurn(90);
                sleep(100);
                gyroTurn(60);
                sleep(250);
                placeGlyph();
                break;
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
