package me.denniss.mvision;

import android.app.Activity;
import android.net.Uri;
import android.os.Bundle;
import android.app.Fragment;
import android.support.v4.widget.DrawerLayout;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;



public class DrawerFragment extends Fragment {

    private View drawerContainer;
    private DrawerLayout drawerLayout;
    private ListView listView;
    private OnDrawerInteractionListener listener;
    private int currentPosition = 0;


    public DrawerFragment() {
        // Required empty public constructor
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        navigate(currentPosition);
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        // Inflate the layout for this fragment
        listView = (ListView) inflater.inflate(R.layout.fragment_drawer, container, false);


        listView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {
                navigate(position);
            }
        });

        String[] titles = new String[] {
                "Home",
                "Tracking",
                "Recording"
        };

        listView.setAdapter(new ArrayAdapter<String>(
                getActivity(),
                android.R.layout.simple_list_item_activated_1,
                android.R.id.text1,
                titles));

        listView.setItemChecked(currentPosition, true);

        return listView;
    }


    public void configure(View drawerContainer, DrawerLayout drawerLayout){
        this.drawerContainer = drawerContainer;
        this.drawerLayout = drawerLayout;

    }


    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        try {
            listener = (OnDrawerInteractionListener) activity;
        } catch (ClassCastException e) {
            throw new ClassCastException(activity.toString()
                    + " must implement OnDrawerInteractionListener");
        }
    }

    @Override
    public void onDetach() {
        super.onDetach();
        listener = null;
    }



    private void navigate(int position){
        currentPosition = position;

        if(listView != null)
            listView.setItemChecked(position, true);

        if(drawerLayout != null)
            drawerLayout.closeDrawer(drawerContainer);

        if(listener != null)
            listener.onDrawerNavigation(position);
    }


    public static interface OnDrawerInteractionListener {
        void onDrawerNavigation(int page);
    }


}
