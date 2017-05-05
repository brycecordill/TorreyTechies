package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;

import java.util.ArrayList;
import java.util.Set;

@Autonomous(name="NewBotAuto9367", group = "9367")
public class NewBotAuto9367 extends LinearOpMode {

    DcMotor flDrive, frDrive, rlDrive, rrDrive, collector, shooter;
    Servo lServo, rServo;
    ColorSensor beaconColorSensor;
    OpticalDistanceSensor beaconDistanceSensor, lineDistanceSensor;


    boolean releaseOneBall = false;


    //need to test out the followings
    double ballServoBlockPosition = 0;
    double ballServoReleasePosition = 1;
    double lServoRestPosition = 1;
    double lServoPressPosition = 0;
    double rServoRestPosition = 0;
    double rServoPressPosition = 1;

    long setTime = 0;
    long ballServoReleaseTime = 1000;
    long ballServoReleaseWaitTime = 500;
    long shooterResetTime = 1000;
    long shooterShootingTime = 1500;
    double lightSensorThreshold = 0.15;
    double lineColorSensorThreshold = 10;
    double encoderErrorTolerance = 5;

    int clicksToPowerRatio = 4000; //need to be tested out
    //  double whiteLineSearchTurningFactor = 3; //need to be tested out

    int initialHeading = 0;
    int lastHeading = initialHeading;
    double headingCorrectionFactor = 200;
    double turningTolerance = 0.5;
    double turningSlowDownThreshold = 20;
    double isPositionReachedThreshold = 5;

    double whiteLineEdge = 0.4;//need to test out
    double beaconStopDistance = 0.2;
    double approachBeaconCorrectionFactor = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        flDrive = hardwareMap.dcMotor.get("flDrive");
        frDrive = hardwareMap.dcMotor.get("frDrive");
        rlDrive = hardwareMap.dcMotor.get("rlDrive");
        rrDrive = hardwareMap.dcMotor.get("rrDrive");
        collector = hardwareMap.dcMotor.get("collector");
        shooter = hardwareMap.dcMotor.get("shooter");
        beaconDistanceSensor = hardwareMap.opticalDistanceSensor.get("beaconDistanceSensor");
        lineDistanceSensor = hardwareMap.opticalDistanceSensor.get("lineDistanceSensor");
        beaconColorSensor = hardwareMap.colorSensor.get("beaconColorSensor");

        frDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rrDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        collector.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        ResetAllEncoders();

        waitForStart();


    }

    void SearchWhiteLine(String team) {

        StopDrives();

        while (lineDistanceSensor.getLightDetected() < whiteLineEdge) {
            if (team.equalsIgnoreCase("red")) {
                Forward(0.5);
            }
            else if(team.equalsIgnoreCase("blue")){
                Backward(0.5);
            }
        }

        StopDrives();
    }

    void ApproachBeacon(){
        while(beaconDistanceSensor.getLightDetected() > beaconStopDistance){
            flDrive.setPower(- 0.4 - (lineDistanceSensor.getLightDetected() - whiteLineEdge) * approachBeaconCorrectionFactor);
            frDrive.setPower(0.4 + (lineDistanceSensor.getLightDetected() - whiteLineEdge) * approachBeaconCorrectionFactor);
            rlDrive.setPower(0.4 - (lineDistanceSensor.getLightDetected() - whiteLineEdge) * approachBeaconCorrectionFactor);
            rrDrive.setPower(- 0.4 + (lineDistanceSensor.getLightDetected() - whiteLineEdge) * approachBeaconCorrectionFactor);
        }
    }

    void SetAllDrivesToRunToPosition() {
        flDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rlDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rrDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void SetAllDrivesToUsingEncoders() {
        flDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void SetAllDrivesToNotUsingEncoders() {
        flDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    void ResetAllEncoders() {
        flDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rlDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rrDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    //stop all drives
    void StopDrives() {

        SetAllDrivesToNotUsingEncoders();
        flDrive.setPower(0);
        frDrive.setPower(0);
        rlDrive.setPower(0);
        rrDrive.setPower(0);

    }

    void WaitFor(long time) {
        setTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - setTime < time) {
            continue;
        }
    }
/*
    void ShootBall(boolean releaseBall) {

        ballServo.setPosition(ballServoBlockPosition);
        shooter.setPower(0);
        collector.setPower(0);

        if (releaseBall) {

            ballServo.setPosition(ballServoReleasePosition);
            collector.setPower(1);

            //wait until time is reached
            WaitFor(ballServoReleaseTime);
            ballServo.setPosition(ballServoBlockPosition);
            collector.setPower(0);

            //wait until ball is in position for shooting
            WaitFor(ballServoReleaseWaitTime);

            shooter.setPower(-0.5);

            WaitFor(shooterResetTime);

            shooter.setPower(1);

            WaitFor(shooterShootingTime);

            shooter.setPower(0);
        }


        //If a ball is already on the ramp
        else {

            shooter.setPower(-0.5);
            WaitFor(shooterResetTime);
            shooter.setPower(1);
            WaitFor(shooterShootingTime);
            shooter.setPower(0);

        }
    }
*/

    // MoveUsingEncoders (constant speed)
    void Forward(double power) {

        SetAllDrivesToUsingEncoders();

        flDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));
        frDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));
        rlDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));
        rrDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));

        flDrive.setPower(power);
        frDrive.setPower(power);
        rlDrive.setPower(power);
        rrDrive.setPower(power);

    }

    void Backward(double power) {

        SetAllDrivesToUsingEncoders();

        flDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));
        frDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));
        rlDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));
        rrDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));

        flDrive.setPower(-power);
        frDrive.setPower(-power);
        rlDrive.setPower(-power);
        rrDrive.setPower(-power);

    }

    void Left(double power) {

        SetAllDrivesToUsingEncoders();

        flDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));
        frDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));
        rlDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));
        rrDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));

        flDrive.setPower(-power);
        frDrive.setPower(power);
        rlDrive.setPower(power);
        rrDrive.setPower(-power);

    }

    void Right(double power) {

        SetAllDrivesToUsingEncoders();

        flDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));
        frDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));
        rlDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));
        rrDrive.setMaxSpeed((int) (Math.abs(power) * clicksToPowerRatio));

        flDrive.setPower(power);
        frDrive.setPower(-power);
        rlDrive.setPower(-power);
        rrDrive.setPower(power);

    }


    boolean isPositionReached() {
        if((Math.abs(flDrive.getCurrentPosition() - flDrive.getTargetPosition()) + Math.abs(frDrive.getCurrentPosition() - frDrive.getTargetPosition()) + Math.abs(rlDrive.getCurrentPosition() - rlDrive.getTargetPosition()) + Math.abs(rrDrive.getCurrentPosition() - rrDrive.getTargetPosition())) / 4 < encoderErrorTolerance){
            return true;
        }
        else{
            return false;
        }
    }


    double IntArrayListAvg(ArrayList<Integer> a) {
        double sum = 0;
        for (int i = 0; i < a.size(); i++) {
            sum += a.get(i);
        }
        return sum / a.size();
    }

    //for beacon detection
    String AnalyzeColor(long detectionTime) {

        ArrayList<Integer> red = new ArrayList<>();
        ArrayList<Integer> blue = new ArrayList<>();

        long initTime = System.currentTimeMillis();

        double redSum = 0;
        double blueSum = 0;

        while (System.currentTimeMillis() - initTime < detectionTime) {
            red.add(beaconColorSensor.red());
            blue.add(beaconColorSensor.blue());
        }

        if (IntArrayListAvg(red) < (IntArrayListAvg(blue) - 2)) {
            return "blue";
        } else {
            return "red";
        }
    }


    void ForwardPosition(double power, int distance) {

        StopDrives();
        SetAllDrivesToRunToPosition();

        flDrive.setTargetPosition(flDrive.getCurrentPosition() + distance);
        frDrive.setTargetPosition(frDrive.getCurrentPosition() + distance);
        rlDrive.setTargetPosition(rlDrive.getCurrentPosition() + distance);
        rrDrive.setTargetPosition(rrDrive.getCurrentPosition() + distance);

        while (!isPositionReached()) {
            flDrive.setPower(power);
            frDrive.setPower(power);
            rlDrive.setPower(power);
            rrDrive.setPower(power);
        }

        StopDrives();

    }

    void BackWardPosition(double power, int distance) {

        StopDrives();
        SetAllDrivesToRunToPosition();

        flDrive.setTargetPosition(flDrive.getCurrentPosition() - distance);
        frDrive.setTargetPosition(frDrive.getCurrentPosition() - distance);
        rlDrive.setTargetPosition(rlDrive.getCurrentPosition() - distance);
        rrDrive.setTargetPosition(rrDrive.getCurrentPosition() - distance);

        while (!isPositionReached()) {
            flDrive.setPower(-power);
            frDrive.setPower(-power);
            rlDrive.setPower(-power);
            rrDrive.setPower(-power);
        }

        StopDrives();

    }

    void LeftPosition(double power, int distance) {

        StopDrives();
        SetAllDrivesToRunToPosition();

        flDrive.setTargetPosition(flDrive.getCurrentPosition() - distance);
        frDrive.setTargetPosition(frDrive.getCurrentPosition() + distance);
        rlDrive.setTargetPosition(rlDrive.getCurrentPosition() + distance);
        rrDrive.setTargetPosition(rrDrive.getCurrentPosition() - distance);

        while (!isPositionReached()) {
            flDrive.setPower(-power);
            frDrive.setPower(power);
            rlDrive.setPower(power);
            rrDrive.setPower(-power);
        }

        StopDrives();

    }

    void RightPosition(double power, int distance) {

        StopDrives();
        SetAllDrivesToRunToPosition();

        flDrive.setTargetPosition(flDrive.getCurrentPosition() + distance);
        frDrive.setTargetPosition(frDrive.getCurrentPosition() - distance);
        rlDrive.setTargetPosition(rlDrive.getCurrentPosition() - distance);
        rrDrive.setTargetPosition(rrDrive.getCurrentPosition() + distance);

        while (!isPositionReached()) {
            flDrive.setPower(power);
            frDrive.setPower(-power);
            rlDrive.setPower(-power);
            rrDrive.setPower(power);
        }

        StopDrives();

    }

    void ClockwisePosition(double power, int distance) {

        StopDrives();
        SetAllDrivesToRunToPosition();

        flDrive.setTargetPosition(flDrive.getCurrentPosition() + distance);
        frDrive.setTargetPosition(frDrive.getCurrentPosition() - distance);
        rlDrive.setTargetPosition(rlDrive.getCurrentPosition() + distance);
        rrDrive.setTargetPosition(rrDrive.getCurrentPosition() - distance);

        while (!isPositionReached()) {
            flDrive.setPower(power);
            frDrive.setPower(-power);
            rlDrive.setPower(power);
            rrDrive.setPower(-power);
        }

        StopDrives();

    }

    void CounterClockwisePosition(double power, int distance) {

        StopDrives();
        SetAllDrivesToRunToPosition();

        flDrive.setTargetPosition(flDrive.getCurrentPosition() - distance);
        frDrive.setTargetPosition(frDrive.getCurrentPosition() + distance);
        rlDrive.setTargetPosition(rlDrive.getCurrentPosition() - distance);
        rrDrive.setTargetPosition(rrDrive.getCurrentPosition() + distance);

        while (!isPositionReached()) {
            flDrive.setPower(-power);
            frDrive.setPower(power);
            rlDrive.setPower(-power);
            rrDrive.setPower(power);
        }

        StopDrives();

    }
}





    /*
    void SearchForWhiteLine(String teamColor) {

        StopDrives();
        lineColorSensor.enableLed(true);

        if (teamColor.equalsIgnoreCase("red")) {

            while (lightSensor.getLightDetected() < lightSensorThreshold) {
                Forward(0.1);
            }

            StopDrives();

            while (getColorSensorAvg(lineColorSensor) < lineColorSensorThreshold) {
                Forward(0.1);
            }

            StopDrives();

            while(lightSensor.getLightDetected() < lightSensorThreshold) {
                CounterClockwiseTurn(0.1);
            }

            StopDrives();

            //might need to move a bit

            while ((distanceSensor1.getLightDetected() + distanceSensor2.getLightDetected()) / 2 < distanceColorDetection) {
                Left(0.1);
            }

            StopDrives();

            if (AnalyzeColor(1000).equalsIgnoreCase("red")) {
                Move("right", 0.2, 700);
                redServo.setPosition(redServoPressPosition);
            }

            else {
                Move("right", 0.2, 700);
                blueServo.setPosition(blueServoPressPosition);
            }

            while ((distanceSensor1.getLightDetected() + distanceSensor2.getLightDetected()) / 2 < distanceBeaconPressing) {
                Left(0.4);
            }

            WaitFor(800);

            Move("right", 0.2, 1200);
            SetAllVariablesToDefault();

        }

        else {

            while(lightSensor.getLightDetected() < lightSensorThreshold) {
                Backward(0.1);
            }

            StopDrives();

            while(getColorSensorAvg(lineColorSensor) < lineColorSensorThreshold) {
                Backward(0.1);
            }

            StopDrives();

            while(lightSensor.getLightDetected() < lightSensorThreshold) {
                ClockwiseTurn(0.1);
            }

            StopDrives();

            //might need to move a bit

            while ((distanceSensor1.getLightDetected() + distanceSensor2.getLightDetected()) / 2 < distanceColorDetection) {
                Left(0.1);
            }

            StopDrives();

            if (AnalyzeColor(1000).equalsIgnoreCase("red")) {
                Move("right", 0.2, 700);
                redServo.setPosition(redServoPressPosition);
            }

            else {
                Move("right", 0.2, 700);
                blueServo.setPosition(blueServoPressPosition);
            }

            while ((distanceSensor1.getLightDetected() + distanceSensor2.getLightDetected()) / 2 < distanceBeaconPressing) {
                Left(0.4);
            }

            WaitFor(800);

            Move("right", 0.2, 1200);
            SetAllVariablesToDefault();

        }


    }
*/





/*
    double[] getCurrentPosition() {
        double[] ary = {flDrive.getCurrentPosition(), frDrive.getCurrentPosition(), rlDrive.getCurrentPosition(), rrDrive.getCurrentPosition()};
        return ary;
    }

    boolean ArraysSimilar(double[] a, double[] b){

        double sumOfDifference = 0;

        for(int i = 0; i < a.length; i++){
            sumOfDifference += Math.abs(Math.abs(a[i]) - Math.abs(b[i]));
        }

        if(sumOfDifference/4 < encoderErrorTolerance){
            return true;
        }

        else{
            return false;
        }

    }

    double[] ArrayAddition(double[] a, double[] b){
        double[] sum = new double[a.length];
        for(int i = 0; i < sum.length; i++){
            sum[i] = a[i] + b[i];
        }
        return sum;
    }

    double ArrayAvg(double[] a){
        double sum = 0;
        for(int i = 0; i < a.length; i++){
            sum += a[i];
        }
        return sum/a.length;
    }
*/





/*
    //must be used with gyro to correct heading
    void ForwardEncoder(double power, int distance){

        StopDrives();
        SetAllDrivesToRunToPosition();

        double rawHeadingError;
        double headingError = 0;

        flDrive.setTargetPosition(flDrive.getCurrentPosition() + distance);
        frDrive.setTargetPosition(frDrive.getCurrentPosition() + distance);
        rlDrive.setTargetPosition(flDrive.getCurrentPosition() + distance);
        rrDrive.setTargetPosition(frDrive.getCurrentPosition() + distance);

        while(!isPositionReached()){

            rawHeadingError = gyro.getHeading() - lastHeading;
            if(Math.abs(rawHeadingError) > 180) {
                headingError = rawHeadingError - Math.signum(rawHeadingError) * 360;
            }

            flDrive.setPower( power - headingError / headingCorrectionFactor);
            frDrive.setPower( power + headingError / headingCorrectionFactor);
            rlDrive.setPower( power - headingError / headingCorrectionFactor);
            rrDrive.setPower( power + headingError / headingCorrectionFactor);
        }

        StopDrives();

    }

    void BackwardEncoder(double power, int distance) {

        StopDrives();
        SetAllDrivesToRunToPosition();

        double rawHeadingError;
        double headingError = 0;

        flDrive.setTargetPosition(flDrive.getCurrentPosition() - distance);
        frDrive.setTargetPosition(frDrive.getCurrentPosition() - distance);
        rlDrive.setTargetPosition(flDrive.getCurrentPosition() - distance);
        rrDrive.setTargetPosition(frDrive.getCurrentPosition() - distance);

        while(!isPositionReached()){

            rawHeadingError = gyro.getHeading() - lastHeading;
            if(Math.abs(rawHeadingError) > 180) {
                headingError = rawHeadingError - Math.signum(rawHeadingError) * 360;
            }

            flDrive.setPower( - power + headingError / headingCorrectionFactor);
            frDrive.setPower( - power - headingError / headingCorrectionFactor);
            rlDrive.setPower( - power + headingError / headingCorrectionFactor);
            rrDrive.setPower( - power - headingError / headingCorrectionFactor);
        }

        StopDrives();

    }

    void LeftEncoder(double power, int distance){

        StopDrives();
        SetAllDrivesToRunToPosition();

        double rawHeadingError;
        double headingError = 0;

        flDrive.setTargetPosition(flDrive.getCurrentPosition() - distance);
        frDrive.setTargetPosition(frDrive.getCurrentPosition() + distance);
        rlDrive.setTargetPosition(flDrive.getCurrentPosition() + distance);
        rrDrive.setTargetPosition(frDrive.getCurrentPosition() - distance);

        while(!isPositionReached()){

            rawHeadingError = gyro.getHeading() - lastHeading;
            if(Math.abs(rawHeadingError) > 180) {
                headingError = rawHeadingError - Math.signum(rawHeadingError) * 360;
            }

            flDrive.setPower( - power + headingError / headingCorrectionFactor);
            frDrive.setPower( power + headingError / headingCorrectionFactor);
            rlDrive.setPower( power - headingError / headingCorrectionFactor);
            rrDrive.setPower( - power - headingError / headingCorrectionFactor);
        }

        StopDrives();

    }

    void RightEncoder(double power, int distance){

        StopDrives();
        SetAllDrivesToRunToPosition();

        double rawHeadingError;
        double headingError = 0;

        flDrive.setTargetPosition(flDrive.getCurrentPosition() + distance);
        frDrive.setTargetPosition(frDrive.getCurrentPosition() - distance);
        rlDrive.setTargetPosition(flDrive.getCurrentPosition() - distance);
        rrDrive.setTargetPosition(frDrive.getCurrentPosition() + distance);

        while(!isPositionReached()){

            rawHeadingError = gyro.getHeading() - lastHeading;
            if(Math.abs(rawHeadingError) > 180) {
                headingError = rawHeadingError - Math.signum(rawHeadingError) * 360;
            }

            flDrive.setPower( power - headingError / headingCorrectionFactor);
            frDrive.setPower( - power - headingError / headingCorrectionFactor);
            rlDrive.setPower( - power + headingError / headingCorrectionFactor);
            rrDrive.setPower( power + headingError / headingCorrectionFactor);
        }

        StopDrives();

    }

    void Rotate(double power, int turningDegree){

        StopDrives();
        SetAllDrivesToNotUsingEncoders();

        double targetDegree = (lastHeading + turningDegree) % 360;
        double errorDegree = turningTolerance + 1;
        double slowDown;

        while(Math.abs(errorDegree) > turningTolerance){

            errorDegree = (targetDegree - gyro.getHeading()) % 360;

            if(Math.abs(errorDegree) > turningSlowDownThreshold){
                flDrive.setPower(Math.signum(targetDegree) * power);
                frDrive.setPower(-Math.signum(targetDegree) * power);
                rlDrive.setPower(Math.signum(targetDegree) * power);
                rrDrive.setPower(-Math.signum(targetDegree) * power);
            }

            else{
                slowDown = Math.abs(errorDegree) / turningSlowDownThreshold;
                flDrive.setPower(Math.signum(targetDegree) * power * slowDown);
                frDrive.setPower(-Math.signum(targetDegree) * power * slowDown);
                rlDrive.setPower(Math.signum(targetDegree) * power * slowDown);
                rrDrive.setPower(-Math.signum(targetDegree) * power * slowDown);
            }
        }

        StopDrives();
        lastHeading = gyro.getHeading();


    }
*/