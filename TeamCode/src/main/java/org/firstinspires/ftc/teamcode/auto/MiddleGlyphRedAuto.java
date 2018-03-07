package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MIDDLE GLYPH Red", group = "Competition")
public class MiddleGlyphRedAuto extends BaseAutonomous {

    @Override
    public void runOpMode() {
        super.runOpMode();


        // Start Moving Here
        switch (detectedVuMark) {
            case UNKNOWN: //Should throw an error but just does left for now.
            case LEFT:
                encoderAccDrive(113, -0.3);
                sleep(500);
                gyroTurn(-110);
                sleep(500);
                encoderAccDrive(17, 0.3);
                sleep(500);
                break;
            case CENTER:
                encoderAccDrive(90, -0.3);
                sleep(500);
                gyroTurn(-110);
                sleep(500);
                encoderAccDrive(20, 0.3);
                sleep(500);
                break;
            case RIGHT:
                encoderAccDrive(105, -0.3);
                sleep(500);
                gyroTurn(-60);
                sleep(500);
                encoderAccDrive(20, 0.3);
                sleep(500);
                break;
        }
        placeGlyph();
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
