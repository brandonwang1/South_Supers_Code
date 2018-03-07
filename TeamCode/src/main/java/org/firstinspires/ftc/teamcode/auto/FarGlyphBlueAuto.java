package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "FAR GLYPH Blue", group = "Competition")
public class FarGlyphBlueAuto extends BaseAutonomous {

    @Override
    public void runOpMode() {
        super.runOpMode();

        placeGlyph();

        /*encoderAccDrive(75,0.3);
        sleep(250);




        switch (detectedVuMark) {
            case UNKNOWN: // Fall through to left case
            case LEFT:
                //moves away from the balancing stone
                gyroTurn(16);
                sleep(250);
                encoderAccDrive(20,0.3);
                placeGlyph();

                break;
            case CENTER:
                //moves away from the balancing stone
                gyroTurn(90);
                sleep(250);
                encoderAccDrive(17,0.3);
                sleep(250);
                gyroTurn(-60);
                sleep(250);
                encoderAccDrive(15,0.3);
                placeGlyph();

                break;
            case RIGHT:
                //moves away from the balancing stone

                gyroTurn(90);
                sleep(250);
                encoderAccDrive(36,0.3);
                sleep(250);
                gyroTurn(-60);
                sleep(250);
                encoderAccDrive(15,0.3);
                placeGlyph();

                break;
        }*/

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
