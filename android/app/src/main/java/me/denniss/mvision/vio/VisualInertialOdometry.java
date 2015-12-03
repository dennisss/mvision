package me.denniss.mvision.vio;

import android.util.Log;


/**
 * Created by dennis on 8/20/15.
 */
public class VisualInertialOdometry {


    public VisualInertialOdometry(){

        // Initialize the native object
        native_inst = ctor();
    }

    public void onStart(){
        nativeStart(native_inst);
    }
    public void onStop(){
        nativeStop(native_inst);
    }



    public void onCameraViewStarted(int width, int height) {
        //mGray = new Mat();
        //mRgba = new Mat();
    }

    public void onCameraViewStopped() {
        //mGray.release();
        //mRgba.release();
    }

    /*
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame frame) {

        Mat img = frame.rgba();

        //Mat res = new Mat();

        //resize(img, res, new Size(640, 360));


        Log.i("VisInOdo", "Size: " + img.height() + " " + img.width());

        // Call the native image update code
        // Make sure that the native code also draws the matched features
        onNativeCameraFrame(native_inst, img.getNativeObjAddr());

        return img;

    }
    */

/*
    public void startRecording(){
        this.nativeStartRecording(native_inst);
    }
    public void stopRecording(){
        this.nativeStopRecording(native_inst);
    }

    private static native void nativeStartRecording(long inst);
    private static native void nativeStopRecording(long inst);
*/



    private long native_inst;
    private static native long ctor();
    private static native void destructor(long inst);

    private static native void nativeStart(long inst);
    private static native void nativeStop(long inst);

    private static native void onNativeCameraFrame(long inst, long img);

}
