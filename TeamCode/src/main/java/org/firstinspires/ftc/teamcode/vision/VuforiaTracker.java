/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.vision;

import android.util.Log;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Constants;

public class VuforiaTracker {

    private ClosableVuforiaLocalizer vuforia; // The Localizer
    private VuforiaTrackable relicTemplate; // Relic Tracker

    /* Constructor */
    public VuforiaTracker() {
    }

    public void initVuforia(HardwareMap ahwMap) {
        /*
         * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
         * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
         */
        int cameraMonitorViewId = ahwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", ahwMap.appContext.getPackageName());
        ClosableVuforiaLocalizer.Parameters parameters = new ClosableVuforiaLocalizer.Parameters(cameraMonitorViewId);
        //ClosableVuforiaLocalizer.Parameters parameters = new ClosableVuforiaLocalizer.Parameters();

        /* Add Vuforia License Key */
        parameters.vuforiaLicenseKey = Constants.VuforiaLicenseKey;

        /* We also indicate which camera on the RC that we wish to use. */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = new ClosableVuforiaLocalizer(parameters);

        // Setup image retrieval
        //vuforia.setFrameQueueCapacity(1);
        //Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);

        /**
         * Load the data set containing the VuMarks for Relic Recovery.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary
        relicTrackables.activate();
        setFlash(false);
    }


    /**
     * See if any of the instances of relicTemplate are currently visible.
     * {@link RelicRecoveryVuMark} is an enum which can have the following values:
     * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
     * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
     */
    public RelicRecoveryVuMark getVisibleVuMark() {
        return RelicRecoveryVuMark.from(relicTemplate);
    }

    public void closeVuforia() {
        setFlash(false);
        vuforia.close();
    }

    private void setFlash(boolean on) {
        vuforia.enableFlash(on);
    }


    // This method + vuforiaImgToMat takes 50 ms to run.
    public Image getImageFromFrame() {
        try {
            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
            long numImgs = frame.getNumImages();
            for (int i = 0; i < numImgs; i++) {
                Log.d("Vuforia", "loop through frames");
                Log.d("Vuforia", String.valueOf(frame.getImage(i).getFormat()));
                if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    return frame.getImage(i);
                }//if
            }//for

        } catch (Exception e) {
            Log.w("Vuforia", "Image taking failed - exception");
        }
        Log.d("Vuforia", "nothing found.");
        return null;
    }


    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
