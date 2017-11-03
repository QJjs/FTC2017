package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

import for_camera_opmodes.OpModeCamera;

/**
 * Created by robolab on 10/12/2017.
 */

@TeleOp(name = "Camera", group = "Prototype")
public class oldCameraTest extends OpModeCamera
{

    @Override
    public void init()
    {
        setCameraDownsampling(4);
        OpenCVLoader.initDebug();
        telemetry.addLine("2");
        super.init();
    }

    @Override
    public void loop()
    {
        if(imageReady())
        {
            Bitmap bm = convertYuvImageToRgb(yuvImage, width, height, 4);
            Mat mat = new Mat();
            //Utils.bitmapToMat(bm, mat);
            //find(mat);

            telemetry.addLine("width: " + width + "px");
            telemetry.addLine("height: " + height + "px");
        }
    }

    @Override
    public void stop()
    {
        super.stop();
    }

    /*void find(Mat frame)
    {
        Mat blurredImg = new Mat(), hsvImg = new Mat(), mask = new Mat(), output = new Mat();
        float hueStart = 20, hueEnd = 50,
                satStart = 60, satEnd = 200,
                valStart = 50, valEnd = 255;
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        Imgproc.blur(frame, blurredImg, new Size(7, 7));
        Imgproc.cvtColor(blurredImg, hsvImg, Imgproc.COLOR_BGR2HSV);

        Scalar minValues = new Scalar(hueStart, satStart, valStart);
        Scalar maxValues = new Scalar(hueEnd, satEnd, valEnd);

        Core.inRange(hsvImg, minValues, maxValues, mask);

        Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(24, 24));
        Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(12, 12));

        Imgproc.erode(mask, output, erodeElement);
        Imgproc.erode(mask, output, erodeElement);
        Imgproc.dilate(mask, output, dilateElement);
        Imgproc.dilate(mask, output, dilateElement);

        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_SIMPLE);

        if(hierarchy.size().width > 0 && hierarchy.size().height > 0)
        {
            telemetry.addData("contours: ", contours.size());
        }
    }*/
}
