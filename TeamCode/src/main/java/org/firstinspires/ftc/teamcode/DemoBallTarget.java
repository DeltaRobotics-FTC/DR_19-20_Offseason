package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.lang.Math;

import for_camera_opmodes.LinearOpModeCamera;

@Autonomous(name = "DemoBallTarget", group = "")
public class DemoBallTarget extends LinearOpModeCamera {

    //This sets the motorXX to DcMotor Objects
    DcMotor motorRF = null;
    DcMotor motorLF = null;
    DcMotor motorRB = null;
    DcMotor motorLB = null;
    DcMotor shooter = null;

    //Target pixel for the X axis of photo
    final int PIXEL_X_TARGET = 624;
    //Target pixel for the Y axis of photo
    final int PIXEL_Y_TARGET = 1232;

    /*

    X axis    ^
     |        | <-- Y axis
     |        |
     V        |
    <---------o <-- target point. Top left corner of ball will be the target point.

     */






    //Conversion factor from pixels to degrees
    final double PIXEL_DEGREES = 0.0224;
    //Conversion factor from pixels to encoder counts
    final double PIXEL_ENCODER = 0.694;

    int ballXPixel = 0;
    int ballYPixel = 0;

    int xPixelDelta = 0;
    int yPixelDelta = 0;

    Drive_MK2 drive = new Drive_MK2();

    BNO055IMU imu;
    Orientation angles;


    @Override
    public void runOpMode() {
        //Maps motor objects to name in robot configuration
        motorRF = hardwareMap.dcMotor.get("motorRF");
        motorRB = hardwareMap.dcMotor.get("motorRB");
        motorLF = hardwareMap.dcMotor.get("motorLF");
        motorLB = hardwareMap.dcMotor.get("motorLB");
        shooter = hardwareMap.dcMotor.get("shooter");

        //Setting motors to brake when stopped
        motorRF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        DcMotor[] motors = new DcMotor[4];
        motors[0] = motorRF;
        motors[1] = motorRB;
        motors[2] = motorLB;
        motors[3] = motorLF;

        BNO055IMU.Parameters parametersIMU = new BNO055IMU.Parameters(); //Declares parameters object forIMU
        parametersIMU.angleUnit = BNO055IMU.AngleUnit.DEGREES; //Sets the unit in which we measure orientation in degrees
        parametersIMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC; //Sets acceleration unit in meters per second ??
        parametersIMU.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode, sets what file the IMU ueses
        parametersIMU.loggingEnabled = true; //Sets wether logging in enable
        parametersIMU.loggingTag = "IMU"; //Sets logging tag
        parametersIMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); //Sets acceleration integration algorithm
        parametersIMU.temperatureUnit = BNO055IMU.TempUnit.CELSIUS; //Sets units for temperature readings
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //Inits IMU
        imu.initialize(parametersIMU); //Init IMU parameters (set above)
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); //Gets current orientation of robot
        telemetry.addData("Init Orientation", AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle)); //Displays initial orientation


        //First picture
        if (isCameraAvailable()) {
            setCameraDownsampling(1);


            startCamera();

            waitForStart();


            if (imageReady()) {

                Bitmap rgbImage;
                //The last value must correspond to the downsampling value from above
                rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);


                SaveImage(rgbImage);

                //Analyzing Jewel Color
                for (int y = 0; y < rgbImage.getHeight(); y++)
                {

                    for (int x = 0; x < rgbImage.getWidth(); x++)
                    {
                        int pixel = rgbImage.getPixel(x, y);
                        if(red(rgbImage.getPixel(x, y)) > 185 && green(rgbImage.getPixel(x, y)) < 20 && blue(rgbImage.getPixel(x, y)) < 35)
                        {
                            ballXPixel = x;
                            ballYPixel = y;
                        }

                    }
                }
                telemetry.addData("Width", rgbImage.getWidth());
                telemetry.addData("Height", rgbImage.getHeight());
                telemetry.addData("X Coord", ballXPixel);
                telemetry.addData("Y Coord", ballYPixel);
                telemetry.addData("Red Pixel", red(rgbImage.getPixel(ballXPixel, ballYPixel)));
                telemetry.addData("Green Pixel", green(rgbImage.getPixel(ballXPixel, ballYPixel)));
                telemetry.addData("Green Pixel", blue(rgbImage.getPixel(ballXPixel, ballYPixel)));

                telemetry.update();
                sleep(1000);
                stopCamera();


            }
        }

        //Sets target deltas for movement
        xPixelDelta = PIXEL_X_TARGET - ballXPixel;
        yPixelDelta = PIXEL_Y_TARGET - ballYPixel;

        //Orientating towards ball
        drive.OrientationDrive((xPixelDelta * PIXEL_DEGREES), 0.6, motors, imu);
        sleep(250);

        //Moving towards ball
        //IMPORTANT------- Changed the encoder delta argument to double for the sake of this program. If causes errors, change back!---------------------
        drive.encoderDrive((yPixelDelta * PIXEL_ENCODER), driveStyle.BACKWARD, 0.8, motors);


            //Second picture
            /*sleep(2000);
            if (isCameraAvailable()) {
                setCameraDownsampling(1);

                startCamera();


                if (imageReady()) {
                    int redValueLeft = -76800;
                    int blueValueLeft = -76800;
                    int greenValueLeft = -76800;

                    Bitmap rgbImage;
                    //The last value must correspond to the downsampling value from above
                    rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);


                    SaveImage(rgbImage);

                    //Analyzing Jewel Color
                    for (int x = 0; x < rgbImage.getWidth(); x++) {
                        for (int y = 0; y < rgbImage.getHeight(); y++) {
                            int pixel = rgbImage.getPixel(x, y);

                        }
                    }
                    telemetry.addData("Width", rgbImage.getWidth());
                    telemetry.addData("Height", rgbImage.getHeight());
                    telemetry.update();
                    sleep(2000);
                    stopCamera();


                }

                drive.OrientationDrive(15, 0.65, motors, imu);
                sleep(2000);

                //Third picture
                if (isCameraAvailable()) {
                    setCameraDownsampling(1);

                    startCamera();


                    if (imageReady()) {
                        int redValueLeft = -76800;
                        int blueValueLeft = -76800;
                        int greenValueLeft = -76800;

                        Bitmap rgbImage;
                        //The last value must correspond to the downsampling value from above
                        rgbImage = convertYuvImageToRgb(yuvImage, width, height, 1);


                        SaveImage(rgbImage);

                        //Analyzing Jewel Color
                        for (int x = 0; x < rgbImage.getWidth(); x++) {
                            for (int y = 0; y < rgbImage.getHeight(); y++) {
                                int pixel = rgbImage.getPixel(x, y);

                            }
                        }
                        telemetry.addData("Width", rgbImage.getWidth());
                        telemetry.addData("Height", rgbImage.getHeight());
                        telemetry.update();
                        sleep(2000);
                        stopCamera();


                    }
                }
            }
           */
        }
    }

