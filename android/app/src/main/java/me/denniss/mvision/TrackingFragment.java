package me.denniss.mvision;

import android.app.Activity;
import android.net.Uri;
import android.os.Bundle;
import android.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;


public class TrackingFragment extends Fragment {

//    private CameraBridgeViewBase mOpenCvCameraView;


    public TrackingFragment() {
        // Required empty public constructor
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

    }




    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        View view =  inflater.inflate(R.layout.fragment_tracking, container, false);

        //mOpenCvCameraView = (CameraBridgeViewBase) view.findViewById(R.id.surfaceView);
        //mOpenCvCameraView.enableFpsMeter();

        return view;
    }


    @Override
    public void onResume(){
        super.onResume();
        //mOpenCvCameraView.enableView();
    }

    @Override
    public void onPause(){
        super.onPause();
        //mOpenCvCameraView.disableView();
    }

}
