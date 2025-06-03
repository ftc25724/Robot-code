package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.GyroSensor;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Omni;


@TeleOp(name="Robot: Arm", group="Robot")

public class Arm extends LinearOpMode {

public Arm(){};

    // Declare OpMode members for each of the motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor slider = null;
    private DcMotor armm = null;
    private DcMotor lifty = null;
    private Servo graby = null;
    private Servo wrist = null;
    // @Override
    public void runOpMode() {
        
        // private ElapsedTime runtime = new ElapsedTime();
        // private DcMotor slider = null;
        // private DcMotor armm = null;
        // private DcMotor lifty = null;
        // private Servo graby = null;
        // private Servo wrist = null;


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        slider = hardwareMap.get(DcMotor.class, "slide");
        armm = hardwareMap.get(DcMotor.class, "arm");
        lifty = hardwareMap.get(DcMotor.class, "lift");
        graby = hardwareMap.get(Servo.class, "grab");
        wrist = hardwareMap.get(Servo.class, "wrist");
        
        int armposition = 0;
        
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifty.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifty.setMode(DcMotor.RunMode.RUN_USING_ENCODER);        
        
        wrist.setPosition(0);
        graby.setPosition(0);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        int slidermax = -5210;
        int armmax = -7610;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            double sliderPower = gamepad2.left_stick_y;
            double armPower = gamepad2.right_stick_y;
            
            
            if (gamepad2.right_trigger > 0){
                slider.setTargetPosition(slidermax);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderPower = -gamepad2.right_trigger;
                armm.setTargetPosition(-7610);
                armm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPower = -gamepad2.right_trigger;
                wrist.setPosition(0.30);
            }
            else if (gamepad2.left_trigger > 0){
                slider.setTargetPosition(-268);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderPower = gamepad2.left_trigger;
                armm.setTargetPosition(-90);
                armm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armPower = gamepad2.left_trigger - 0.15;
                wrist.setPosition(0);
                graby.setPosition(0);
            }
            else if (gamepad2.left_stick_y < -0.05){
                slider.setTargetPosition(slidermax);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad2.left_stick_y >= 0.05){
                slider.setTargetPosition(-268);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad2.right_stick_y < -0.05){
                armm.setTargetPosition(-7610);
                armm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad2.right_stick_y >= 0.05){
                armm.setTargetPosition(-90);
                armm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else{
                armPower = 0;
                sliderPower = 0;
                
            }
            
            if (armm.getCurrentPosition() <= -7610) {
                slidermax = -5494;
            }
            else if (armm.getCurrentPosition() > -7610) {
                slidermax = -5210;
            }
            
            if (lifty.getCurrentPosition() > 10) {
                armmax = -9799;
            }
            else if (lifty.getCurrentPosition() <= 10) {
                armmax = -7610;
            }

            else if (gamepad2.left_bumper){
                graby.setPosition(0.10);
            }
    
            if (gamepad2.dpad_up) {
                wrist.setPosition(0.3);
            }
            else if (gamepad2.dpad_right) {
                wrist.setPosition(0.15);
            }
            else if (gamepad2.dpad_down) {
                wrist.setPosition(0);
            }

            if (gamepad1.dpad_up) {
                lifty.setTargetPosition(28000);
                lifty.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.dpad_down) {
                lifty.setTargetPosition(0);
                lifty.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            else if (gamepad1.dpad_right) {
                lifty.setTargetPosition(12000);
                lifty.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if (lifty.getCurrentPosition() < (lifty.getTargetPosition() - 5)) {
                lifty.setPower(100);
            }
            else if (lifty.getCurrentPosition() > (lifty.getTargetPosition() + 5)){
                lifty.setPower(-100);
            }
            else{
                lifty.setPower(0);
            }

            if (gamepad1.y) {
                lifty.setTargetPosition(0);
                slider.setTargetPosition(-3721);
                
            }

            // Send calculated power to wheel
            slider.setPower(sliderPower);
            armm.setPower(armPower);
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("slide", slider.getCurrentPosition());
            telemetry.addData("arm", armm.getCurrentPosition());
            telemetry.addData("lift", lifty.getCurrentPosition());
            telemetry.addData("grab", graby.getPosition());
            telemetry.addData("wrist", wrist.getPosition());
            telemetry.addData("out  ", "%4.2f", gamepad2.left_stick_y);
            telemetry.addData("rt", gamepad2.right_trigger);
            telemetry.addData("lt", gamepad2.left_trigger);
            // telemetry.addData("speed", varspeed);
            telemetry.update();
        }
    }}
    