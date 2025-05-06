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
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontL  = hardwareMap.get(DcMotor.class, "flm");
        backL  = hardwareMap.get(DcMotor.class, "blm");
        frontR = hardwareMap.get(DcMotor.class, "frm");
        backR = hardwareMap.get(DcMotor.class, "brm");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        
        int armposition = 0;
        
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                
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

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);            
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial    = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral  =  gamepad1.left_stick_x;
            double yaw      =  gamepad1.right_stick_x * 1.1;
            double lateralx = lateral * Math.cos(-botHeading) - axial * Math.sin(-botHeading);
            double axialy   = lateral * Math.sin(-botHeading) + axial * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(axialy) + Math.abs(lateralx) + Math.abs(yaw), 1);
                            
            double leftFrontPower = (axialy + lateralx + yaw) / denominator;
            double leftBackPower = (axialy - lateralx + yaw) / denominator;
            double rightFrontPower = (axialy - lateralx - yaw) / denominator;
            double rightBackPower = (axialy + lateralx - yaw) / denominator;
                
            lateralx = lateralx * 1.1;
                
            // Send calculated power to wheels
            frontL.setPower(leftFrontPower);
            frontR.setPower(rightFrontPower);
            backL.setPower(leftBackPower);
            backR.setPower(rightBackPower);
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Axial  ", "%4.2f", axial);
            telemetry.addData("lateral  ", "%4.2f", lateral);
            telemetry.addData("yaw  ", "%4.2f", yaw);
            telemetry.update();
        }
    }}