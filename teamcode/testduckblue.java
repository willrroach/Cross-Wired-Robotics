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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="testduckblue", group="Linear Opmode")
//@Disabled
public class testduckblue extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Motors
    private DcMotor leftDriveBack;
    private DcMotor leftDriveFront;
    private DcMotor rightDriveBack;
    private DcMotor rightDriveFront;
    private DcMotor collectionMotor;
    private DcMotor carouselMotor;
    private DcMotor viperMotor;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Maps all of the hardware to the software variable names. The green names in "" are what appear on the
        //Driver hub config. settings. These must match exactly
        rightDriveFront = hardwareMap.get(DcMotor.class, "FRW");
        leftDriveFront = hardwareMap.get(DcMotor.class, "FLW");
        rightDriveBack = hardwareMap.get(DcMotor.class, "BRW");
        leftDriveBack = hardwareMap.get(DcMotor.class, "BLW");
        collectionMotor = hardwareMap.get(DcMotor.class, "COLLM");
        carouselMotor = hardwareMap.get(DcMotor.class, "CARM");
        viperMotor = hardwareMap.get(DcMotor.class, "VIPM");

        //Sets the directions of all DC Motors
        leftDriveBack.setDirection(DcMotor.Direction.REVERSE);
        leftDriveFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotorSimple.Direction.FORWARD);
        collectionMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        viperMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            translate(0.2, 1000, true);
            sleep(100);
            moveStraight(-0.2, 1000);
            sleep(100);
            spinCarousel(true, 3800);
            sleep(100);
            moveStraight(0.5, 1000);
            sleep(100);
            translate(0.2, 2800, false);
            sleep(100);
            moveStraight(0.4, 3500);
            sleep(30000);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Front Position:", leftDriveFront.getCurrentPosition());
            telemetry.addData("Right Front Position:", rightDriveFront.getCurrentPosition());
            telemetry.addData("Right Back Position:", rightDriveBack.getCurrentPosition());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
        }
    }


    private void moveStraight(double power, int distance) {

        rightDriveFront.setTargetPosition(distance);
        leftDriveFront.setTargetPosition(distance);
        rightDriveBack.setTargetPosition(distance);
        leftDriveBack.setTargetPosition(distance);

        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightDriveFront.setPower(power);
        leftDriveFront.setPower(power);
        rightDriveBack.setPower(power);
        leftDriveBack.setPower(power);

        while (rightDriveFront.isBusy()) {

        }

        motorsOff();
        resetEncoders();
    }

    private void translate(double power, int distance, boolean right) {

        if (right) {
            rightDriveFront.setTargetPosition(-distance);
            leftDriveFront.setTargetPosition(distance);
            rightDriveBack.setTargetPosition(distance);
            leftDriveBack.setTargetPosition(-distance);
        } else {
            rightDriveFront.setTargetPosition(distance);
            leftDriveFront.setTargetPosition(-distance);
            rightDriveBack.setTargetPosition(-distance);
            leftDriveBack.setTargetPosition(distance);
        }

        rightDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightDriveFront.setPower(power);
        leftDriveFront.setPower(power);
        rightDriveBack.setPower(power);
        leftDriveBack.setPower(power);

        while (rightDriveFront.isBusy()) {

        }

        motorsOff();
        resetEncoders();

    }

    private void resetEncoders() {
        rightDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void motorsOff() {
        rightDriveFront.setPower(0);
        leftDriveFront.setPower(0);
        rightDriveBack.setPower(0);
        leftDriveBack.setPower(0);
        collectionMotor.setPower(0);
        carouselMotor.setPower(0);
        viperMotor.setPower(0);
    }

    private void runWithoutEncoders() {
        rightDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    //Same thing but with a clockwise parameter. Also, sleep means that you just do whatever you've been doing
    //for a certain amount of time, thus the "durationMilli"
    private void spinCarousel(boolean clockWise, long durationMilli) {

        runWithoutEncoders();

        int direction;

        if (clockWise) {
            direction = -1;
        } else {
            direction = 1;
        }

        carouselMotor.setPower(0.3 * direction);
        rightDriveFront.setPower(-0.1);
        rightDriveBack.setPower(-0.1);
        leftDriveFront.setPower(-0.1);
        leftDriveBack.setPower(-0.1);

        sleep(durationMilli);

        motorsOff();
        resetEncoders();
    }
}