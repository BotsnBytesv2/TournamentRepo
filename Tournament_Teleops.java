/*
Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
**/


@TeleOp(name = "Tournament_Teleops", group = "Concept")


public class Tournament_Teleops extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
   
    private DcMotor frontleftmotor = null;
    private DcMotor frontrightmotor = null;
    private DcMotor backleftmotor = null;
    private DcMotor backrightmotor = null;
    private DcMotor hanging1 = null;
    private DcMotor hanging2 = null;
    private DcMotor directMotor = null;
    private CRServo downServo = null;
    private CRServo upServo = null;
    private CRServo intake = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        
       
        frontleftmotor = hardwareMap.get(DcMotor.class, "frontleftmotor");
        frontrightmotor = hardwareMap.get(DcMotor.class, "frontrightmotor");
        backrightmotor = hardwareMap.get(DcMotor.class, "backrightmotor");
        backleftmotor = hardwareMap.get(DcMotor.class, "backleftmotor");
        hanging1 = hardwareMap.get(DcMotor.class, "hanging1");
        hanging2 = hardwareMap.get(DcMotor.class, "hanging2");
        directMotor = hardwareMap.get(DcMotor.class, "directmotor");
        upServo = hardwareMap.get(CRServo.class, "upservo");
        downServo = hardwareMap.get(CRServo.class, "downservo");
        intake = hardwareMap.get(CRServo.class,"intake");
        
        frontleftmotor.setPower(0.0);
        frontrightmotor.setPower(0.0);
        backleftmotor.setPower(0.0);
        backrightmotor.setPower(0.0);
        hanging1.setPower(0.0);
        hanging2.setPower(0.0);
        directMotor.setPower(0.0);
        upServo.setPower(0.0);
        downServo.setPower(0.0);
        
        
        
        
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double rightMove;
            double leftMove;
            double directPower;
            double servoPower;

            
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            
            rightMove = -gamepad1.right_stick_y;
            leftMove = gamepad1.left_stick_y;
            directPower = -gamepad2.left_stick_y;
            
            
            
            if(gamepad1.left_trigger>0){
                frontleftmotor.setPower(gamepad1.left_trigger);
                backleftmotor.setPower(-gamepad1.left_trigger);
                frontrightmotor.setPower(gamepad1.left_trigger);
                backrightmotor.setPower(-gamepad1.left_trigger);
            }
            else if(gamepad1.right_trigger>0){
                frontleftmotor.setPower(-gamepad1.right_trigger);
                backleftmotor.setPower(gamepad1.right_trigger);
                frontrightmotor.setPower(-gamepad1.right_trigger);
                backrightmotor.setPower(gamepad1.right_trigger);
            }
            else{
                frontleftmotor.setPower(leftMove);
                backleftmotor.setPower(leftMove);
                frontrightmotor.setPower(rightMove);
                backrightmotor.setPower(rightMove);
            }
            
            
            if(gamepad2.right_bumper){
                hanging1.setPower(-0.75);
                hanging2.setPower(0.75);
            }
            else if(gamepad2.left_bumper){
                hanging1.setPower(0.75);
                hanging2.setPower(-0.75);
            }
            else{
                hanging1.setPower(0.0);
                hanging2.setPower(0.0);
            }
            
            if(gamepad2.a == true){
                servoPower = 0.8;
                upServo.setPower(-servoPower);
                downServo.setPower(servoPower);
                telemetry.addLine("Going Up Now");
                telemetry.addData("Power of Both Servos now", servoPower);
                
            }
            else if(gamepad2.b == true){
                servoPower= -0.8;
                upServo.setPower(-servoPower);
                downServo.setPower(servoPower);
                telemetry.addLine("Going Down Now");
                telemetry.addData("Power of Both Servos now", servoPower);
            }
            else{
                servoPower = 0.0;
                upServo.setPower(servoPower);
                downServo.setPower(servoPower);
                telemetry.addLine("Neither are moving");
            }
            /*if(directPower>0.0 && directPower<=0.5){
                directMotor.setPower(directPower * 0.5);
            }
            else if(directPower<0.0 && directPower>=-0.5){
                directMotor.setPower(directPower * 0.5);
            }
            else if(directPower>0.5){
                directMotor.setPower(directPower*0.75);
            }
            else if(directPower<-0.5){
                directMotor.setPower(directPower*0.75);
            }
            else{
                directMotor.setPower(0.0);
            }*/
            if(gamepad2.dpad_up == true){
                directMotor.setPower(0.5);
                sleep(250);
                directMotor.setPower(0.0);
                telemetry.addLine("Arm is going up");
            }
            else if(gamepad2.dpad_down == true){
                directMotor.setPower(-0.5);
                sleep(250);
                directMotor.setPower(0.0);
                telemetry.addLine("Arm is going down");
            }
            else{
                if(directPower>0.0 && directPower<=0.5){
                    directMotor.setPower(directPower * 0.25);
                }
                else if(directPower<0.0 && directPower>=-0.5){
                    directMotor.setPower(directPower * 0.25);
                }
                else if(directPower>0.5){
                    directMotor.setPower(directPower*0.5);
                }
                else if(directPower<-0.5){
                    directMotor.setPower(directPower*0.5);
                }
                else{
                    directMotor.setPower(0.0);
                    telemetry.addLine("The input arm is not moving");
                }
            }
             if(gamepad2.right_trigger>0.0){
                intake.setPower(0.7);
            }
            else if (gamepad2.left_trigger>0.0){
                intake.setPower(-0.7);
            }
            else{
                intake.setPower(0.0);
            }
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "one (%.2f), two (%.2f), servoone (%.2f), servotwo (%.2f)", onePower, twoPower, servo1pos, servo2pos);
            telemetry.addData("Motors", "left side (%.2f), right side (%.2f)", leftMove, rightMove);
            telemetry.update();
        }
    }
}
