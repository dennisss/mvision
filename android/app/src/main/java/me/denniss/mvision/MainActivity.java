package me.denniss.mvision;

import android.app.Activity;
import android.app.Fragment;
import android.app.FragmentManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.widget.CompoundButton;
import android.widget.SlidingDrawer;
import android.widget.ToggleButton;
import android.support.v4.widget.DrawerLayout;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

import org.opencv.core.Mat;

import me.denniss.mvision.vio.VisualInertialOdometry;

public class MainActivity extends Activity implements DrawerFragment.OnDrawerInteractionListener {

    private static final String TAG = "MainActivity";



    // https://github.com/googlesamples/android-Camera2Basic/blob/master/Application/src/main/java/com/example/android/camera2basic/Camera2BasicFragment.java

    private VisualInertialOdometry vio;


    // TODO: Having OpenCV directly linked doesn't seem necessary as it is separately built into libvio anyway

    private BaseLoaderCallback  mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                {
                    Log.i(TAG, "OpenCV loaded successfully");

                    // Load native library after(!) OpenCV initialization
                    //System.loadLibrary("detection_based_tracker");

                    System.loadLibrary("vio");


                    //vio = new VisualInertialOdometry();
                    //mOpenCvCameraView.setCvCameraViewListener(vio);

                    //vio.onStart();

                    //mOpenCvCameraView.enableView();

                } break;
                default:
                {
                    super.onManagerConnected(status);
                } break;
            }
        }
    };



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        setContentView(R.layout.activity_main);

        getActionBar().hide();


        DrawerLayout drawerLayout = (DrawerLayout) findViewById(R.id.drawerLayout);
        DrawerFragment drawerFrag = (DrawerFragment) getFragmentManager().findFragmentById(R.id.drawer);
        drawerFrag.configure(findViewById(R.id.drawer), drawerLayout);

    }


    @Override
    public void onDrawerNavigation(int page){
        Fragment f = null;

        switch(page){
            case 0: // Home
                //f = new MainFragment();
                break;
            case 1: // Settings
                f = new TrackingFragment();
                break;
            case 2: // PID
                f = new RecordingFragment();
                break;
        }

        if(f == null){
            Log.e("MainActivity", "Invalid page specified");
            return;
        }

        FragmentManager fragmentManager = getFragmentManager();
        fragmentManager.beginTransaction().replace(R.id.container, f).commit();

    }



    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }


    @Override
    public void onResume(){
        super.onResume();

        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG, "Internal OpenCV library not found. Using OpenCV Manager for initialization");
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_0_0, this, mLoaderCallback);
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    @Override
    public void onDestroy(){
        super.onDestroy();


        //mOpenCvCameraView.disableView();
    }

    @Override
    public void onPause()
    {
        super.onPause();
        //if (mOpenCvCameraView != null)
        //    mOpenCvCameraView.disableView();
    }





}