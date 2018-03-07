package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "MIDDLE GLYPH Blue", group = "Competition")
public class MiddleGlyphBlueAuto extends BaseAutonomous {

    @Override
    public void runOpMode() {
        super.runOpMode();


        // Start Moving Here
        switch (detectedVuMark) {
            case UNKNOWN: //Should throw an error but just does left for now.
            case LEFT:
                encoderAccDrive(58, 0.3);
                sleep(500);
                gyroTurn(-70);
                sleep(500);
                encoderAccDrive(20, 0.3);
                break;
            case CENTER:
                encoderAccDrive(79, 0.3);
                sleep(500);
                gyroTurn(-70);
                sleep(500);
                encoderAccDrive(20, 0.3);
                break;
            case RIGHT:
                encoderAccDrive(102, 0.3);
                sleep(500);
                gyroTurn(-70);
                sleep(500);
                encoderAccDrive(26, 0.3);
                break;
        }

        sleep(500);
        placeGlyph();
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
