package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
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

@TeleOp(name="Robot: Omni Tank", group="Robot")

public class OmniTest extends LinearOpMode {

    // Declare OpMode members for each of the motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontL = null;
    private DcMotor backL = null;
    private DcMotor frontR = null;
    private DcMotor backR = null;
    private DcMotor slider = null;
    private DcMotor armm = null;
    private DcMotor lifty = null;
    private Servo graby = null;
    private Servo wrist = null;
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontL  = hardwareMap.get(DcMotor.class, "flm");
        backL  = hardwareMap.get(DcMotor.class, "blm");
        frontR = hardwareMap.get(DcMotor.class, "frm");
        backR = hardwareMap.get(DcMotor.class, "brm");
        slider = hardwareMap.get(DcMotor.class, "slide");
        armm = hardwareMap.get(DcMotor.class, "arm");
        lifty = hardwareMap.get(DcMotor.class, "lift");
        graby = hardwareMap.get(Servo.class, "grab");
        wrist = hardwareMap.get(Servo.class, "wrist");
        myOtos = hardwareMap.get(SparkFunOTOS.class, "sensor_otos");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        
        int armposition = 0;
        
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifty.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifty.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        wrist.setPosition(0);
        graby.setPosition(0);
        
        frontL.setDirection(DcMotor.Direction.REVERSE);
        backL.setDirection(DcMotor.Direction.REVERSE);
        frontR.setDirection(DcMotor.Direction.FORWARD);
        backR.setDirection(DcMotor.Direction.FORWARD);
        
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        final int FAST = 100;
        final int MED = 75;
        final int SLOW = 50;

        int slidermax = -5210;
        int armmax = -7610;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);            
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x * 1.1;
            double lateralx = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
            double axialy = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(axialy) + Math.abs(lateralx) + Math.abs(yaw), 1);
            
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            /*                              double leftInches, double rightInches,
            double timeoutS) {
                double leftFrontPower  = axial + lateral + yaw;
                double rightFrontPower = axial - lateral - yaw;
                double leftBackPower   = axial - lateral + yaw;
                double rightBackPower  = axial + lateral - yaw;
                */
                
                double leftFrontPower = (axialy + lateralx + yaw) / denominator;
                double leftBackPower = (axialy - lateralx + yaw) / denominator;
                double rightFrontPower = (axialy - lateralx - yaw) / denominator;
                double rightBackPower = (axialy + lateralx - yaw) / denominator;
                
                lateralx = lateralx * 1.1;
                
                
                double sliderPower = gamepad2.left_stick_y;
                double armPower = gamepad2.right_stick_y;
                
                if (gamepad1.options) {
                    imu.resetYaw();
            }

            if (gamepad2.right_trigger > 0){
                slider.setTargetPosition(slidermax);
                slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderPower = -gamepad2.right_trigger;
                armm.setTargetPosition(armmax);
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
                armm.setTargetPosition(armmax);
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
            
            /*
            if (armm.getCurrentPosition() <= -7610) {
                slidermax = -6400;
            }
            else if (armm.getCurrentPosition() > -7610) {
                slidermax = -5210;
            }
            
            if (lifty.getCurrentPosition() > 10) {
                armmax = -9325;
            }
            else if (lifty.getCurrentPosition() <= 10) {
                armmax = -7610;
            }
            */
            
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motigrabyon.
            double max = 0;
            max = Math.max(max, Math.abs(leftFrontPower));
            max = Math.max(max, Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            
            
            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            
            if (gamepad2.right_bumper){
                graby.setPosition(0.40);
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
                lifty.setTargetPosition(19000);
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
                lifty.setTargetPosition(2000);
                slider.setTargetPosition(-3721);
                lifty.setPower(100);
                slider.setPower(25);
            }
            
            if (lifty.getTargetPosition() == 2000) {
                if (lifty.getCurrentPosition() <= 2000) {
                    lifty.setPower(0);
                    slider.setPower(0);    
                }
            }

            // Send calculated power to wheels
            frontL.setPower(leftFrontPower);
            frontR.setPower(rightFrontPower);
            backL.setPower(leftBackPower);
            backR.setPower(rightBackPower);
            slider.setPower(sliderPower);
            armm.setPower(armPower);
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("slide", slider.getCurrentPosition());
            telemetry.addData("arm", armm.getCurrentPosition());
            telemetry.addData("lift", lifty.getCurrentPosition());
            telemetry.addData("grab", graby.getPosition());
            telemetry.addData("wrist", wrist.getPosition());
            telemetry.addData("Axial  ", "%4.2f", axial);
            telemetry.addData("lateral  ", "%4.2f", lateral);
            telemetry.addData("yaw  ", "%4.2f", yaw);
            telemetry.addData("up/down", gamepad2.right_stick_y);
            telemetry.addData("out  ", "%4.2f", gamepad2.left_stick_y);
            telemetry.addData("rt", gamepad2.right_trigger);
            telemetry.addData("lt", gamepad2.left_trigger);
            telemetry.update();
        }
    }}