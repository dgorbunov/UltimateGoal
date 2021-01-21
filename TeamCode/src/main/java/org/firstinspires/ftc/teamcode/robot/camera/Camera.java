/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.robot.camera;

import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Controller;
import org.firstinspires.ftc.teamcode.robot.camera.libs.OpenCVController;
import org.firstinspires.ftc.teamcode.robot.camera.libs.VuforiaController;
import org.openftc.easyopencv.OpenCvCameraFactory;

/**
 * In this sample, we demonstrate how to use EasyOpenCV in
 * Vuforia passthrough mode. In this mode, EasyOpenCV does not
 * take direct control over the camera. Instead, it pulls frames
 * out of a VuforiaController's frame queue. This allows you to
 * run both OpenCV and Vuforia simultaneously on the same camera.
 * The downside is that you do not get to choose the resolution
 * of frames delivered to your pipeline, and you do not get any
 * sort of manual control over sensor parameters such as exposure,
 * gain, ISO, or frame rate.
 */
public class Camera implements Controller {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    private org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer vuforia = null;
    private OpenCVController openCV;
    private VuforiaController vuforiaController;

    private boolean useLocalizer = false;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    @Override
    public void init() {

        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCV,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        vuforiaController = new VuforiaController(hardwareMap, telemetry, viewportContainerIds, useLocalizer);
        openCV = new OpenCVController(vuforiaController.getVuforia(), vuforiaController.getParameters(), viewportContainerIds, telemetry);

        vuforiaController.init();
        openCV.init();
    }

    public int getRingCount(){
        return openCV.getRingCount();
    }

    public String getRingCountStr() {
        return openCV.getRingCountStr();
    }

    @Nullable
    public Pose2d getRobotPosition() {
        return vuforiaController.getRobotPosition();
    }

    @Override
    public void start() {
        vuforiaController.start();
        openCV.start();
    }

    @Override
    public void stop() {
        vuforiaController.stop();
        openCV.stop();
    }
}