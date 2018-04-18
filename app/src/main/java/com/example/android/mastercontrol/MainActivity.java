package com.example.android.mastercontrol;

import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Location;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.RelativeLayout;
import android.widget.TextView;

import java.io.File;
import java.io.FileOutputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedList;

import static android.R.attr.min;
import static android.R.attr.mode;
import static android.R.attr.name;
import static com.example.android.mastercontrol.MainActivity.Robots.CARLA;
import static com.example.android.mastercontrol.MainActivity.Robots.CARLETON;
import static com.example.android.mastercontrol.MainActivity.Robots.CARLITO;
import static com.example.android.mastercontrol.MainActivity.Robots.CARLOS;
import static com.example.android.mastercontrol.MainActivity.Robots.CARLY;
import static com.example.android.mastercontrol.MainActivity.Robots.DOC;
import static com.example.android.mastercontrol.MainActivity.Robots.MR;
import static com.example.android.mastercontrol.MainActivity.Robots.MRS;
import static java.util.Arrays.asList;

public class MainActivity extends AppCompatActivity implements SensorEventListener {

    //grid variables
    public boolean gridMode = true;
    public boolean autoMode = false;

    //variables for logging
    float[] mGrav;
    float[] mAcc;
    float[] mGyro;
    float[] mGeo;
    File rrFile;
    File jpgFile;
    File recordingFile;
    FileOutputStream fosRR;
    Boolean logging;
    Location dest_loc;
    double[][] map;
    double [][] orig_map = new double[4][4];
    ArrayList<Location> waypoints;
    Location[][] gridLocations;
    int currRow;
    int currCol;

    String TAG1 = "MASTER";

    //Master variables
    String fromMinion = "", //strings received from WiFi router
            toMinion = "";  //strings to send through WiFi router
    enum Robots {
        DOC, MR, MRS, CARLITO, CARLOS, CARLY, CARLA, CARLETON,
    }
    Robots minion; //initialize minion
    double[] gps_coords;    //initialize minion gps

    boolean initialFieldScan = true,
            isObstacleFound = false,
            isMannequinFound = false;

    double[] destinationLoc = new double[2];

    double[][] gpsList = new double[25][2];
    double[][] searchingList = new double[25][2];
    //ArrayList<double[]>searchingList=new ArrayList<double[]>(25);
    double[][] confirmedList = new double[25][2];





    int gpsListCounter = 0;
    String gpsListString = "";
    String confirmedString = "";


    Robot doc = new Robot(DOC, gps_coords, true);
    Robot mr = new Robot(MR, gps_coords, true);
    Robot mrs = new Robot(MRS, gps_coords, true);
    Robot carlito = new Robot(CARLITO, gps_coords, true);
    Robot carlos = new Robot(CARLOS, gps_coords, false);
    Robot carly = new Robot(CARLY, gps_coords, false);
    Robot carla = new Robot(CARLA, gps_coords, false);
    Robot carleton = new Robot(CARLETON, gps_coords, false);

    Robot robot;

    LinkedList<Robot> free_robots=new LinkedList<>();


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);


        RelativeLayout.LayoutParams lp = new RelativeLayout.LayoutParams(700, 700);

        //add functionality to autoMode button
        Button buttonAuto = (Button) findViewById(R.id.btnAuto);
        buttonAuto.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {
                if (!autoMode) {
                    v.setBackgroundResource(R.drawable.button_auto_on);
                    autoMode = true;
                } else {
                    v.setBackgroundResource(R.drawable.button_auto_off);
                    autoMode = false;
                }

                Log.i("auto",""+autoMode);
            }
        });

        free_robots.add(doc);
        free_robots.add(mr);
        free_robots.add(mrs);
        free_robots.add(carlito);
        free_robots.add(carlos);
        free_robots.add(carly);
        free_robots.add(carla);
        free_robots.add(carleton);
    }

    @Override
    public final void onAccuracyChanged(Sensor sensor, int accuracy) {
        // Do something here if sensor accuracy changes.
    }

    //Called whenever the value of a sensor changes
    @Override
    public final void onSensorChanged(SensorEvent event) {
        if (event.sensor.getType() == Sensor.TYPE_GRAVITY)
            mGrav = event.values;
        if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE)
            mGyro = event.values;
        if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER)
            mAcc = event.values;
        if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD)
            mGeo = event.values;



        receive_from_m();       //place string from wifi network in function here
        Robot assign_mission;
        while(!free_robots.isEmpty()){
            assign_mission=free_robots.pop();
            assign_mission.isSearching=true;
            setClosestObjectDistance(assign_mission);

        }


        updateDisplay();

    }

    private void displayMessageToList(String message) {
        TextView oderSummaryTextView = (TextView) findViewById(R.id.gps_string_tv);
        oderSummaryTextView.setText(message);
    }

    private void displayMessageToConfirmedList(String message) {
        TextView oderSummaryTextView = (TextView) findViewById(R.id.gps_string_tv);
        oderSummaryTextView.setText(message);
    }
    /**************************************************************************************************/
    //receives info in form of string & parses to variables' appropriate types
    void receive_from_m (String data) {
        String string_name = data.substring(data.indexOf("NAME"),data.indexOf("GPS")),
                string_gps = data.substring(data.indexOf("GPS"),data.indexOf("OBS")),
                string_obs = data.substring(data.indexOf("OBS"),data.indexOf("MANN")),
                string_mann = data.substring(data.indexOf("MANN"),data.indexOf("SEARCH")),
                string_search = data.substring(data.indexOf("SEARCH"),data.indexOf("LIDAR")),
                string_lidar = data.substring(data.indexOf("LIDAR"),data.indexOf("LGPS")),
                string_lidarGPS = data.substring(data.indexOf("LGPS"),data.length());

        //get GPS coordinates from message string
        gps_coords = getCoords(string_gps);

        //get obstacle boolean
        isObstacleFound = string_obs.contains("true");

        //get mannequin boolean
        isMannequinFound = string_mann.contains("true");

        //get name of minion that sent string
        string_name = string_name.substring(string_name.indexOf(":")+2,string_name.indexOf(","));

        switch (string_name) {
            case "DOC":
                robot = doc;
                doc.setLocation(gps_coords);
                doc.setStatus(isMannequinFound);
                break;
            case "MR":
                robot = mr;
                mr.setLocation(gps_coords);
                mr.setStatus(isMannequinFound);
                break;
            case "MRS":
                robot = mrs;
                mrs.setLocation(gps_coords);
                mrs.setStatus(isMannequinFound);
                break;
            case "CARLITO":
                robot = carlito;
                carlito.setLocation(gps_coords);
                carlito.setStatus(isMannequinFound);
                break;
            case "CARLOS":
                robot = carlos;
                carlos.setLocation(gps_coords);
                carlos.setStatus(isMannequinFound);
                break;
            case "CARLY":
                robot = carly;
                carly.setLocation(gps_coords);
                carly.setStatus(isMannequinFound);
                break;
            case "CARLA":
                robot = carla;
                carla.setLocation(gps_coords);
                carla.setStatus(isMannequinFound);
                break;
            case "CARLETON":
                robot = carleton;
                carleton.setLocation(gps_coords);
                carleton.setStatus(isMannequinFound);
                break;
            default:
                Log.i("ERROR","Invalid robot name. Refer to Robot name list in code.");
        }

        /*
        //get LIDAR boolean
        hasLIDAR = string_lidar.contains("true");

        //add LIDAR gps calculation of victim to list of gps coordinates
        if (hasLIDAR) {
            if (gpsListCounter < gpsList[0].length) {
                gpsList[gpsListCounter] = getCoords(string_lidarGPS);
                gpsListCounter++;
            } else {
                Log.i(TAG1,"gpsList is full!");
            }
        }

        */
    }

    void updateDisplay () {
        if (initialFieldScan) {
            // update GPS locations list string
            for (int i=0; i<gpsList[0].length; i++) {
                gpsListString = gpsListString + "[LAT:" + gpsList[0] + ", LON:" + gpsList[1] + "] \n";
            }

            displayMessageToList(gpsListString);
        } else {
            if (isMannequinFound) {
                // match GPS coords that were found w/ coords on the list
                for (int i=0; i<searchingList.length; i++) {
                    if (searchingList[i][0] != 0 && searchingList[i][1] != 0 && searchingList[i][0] >= gps_coords[0]-0.000008993     && searchingList[i][0] <= gps_coords[0]+0.000008993   && searchingList[i][1] >= gps_coords[1]-0.000008993 &&searchingList[i][1] <= gps_coords[1]+0.000008993) {
                        // add GPS coords to confirmed list
                        confirmedList[i][0] = searchingList[i][0];
                        confirmedList[i][1] = searchingList[i][1];

                        // erase GPS coords from unconfirmed list///////////////////////////////////////////////////////////////////
                        searchingList[i][0] = 0;
                        searchingList[i][1] = 0;

                    }
                }

                // update confirmed String
                for (int i=0; i<confirmedList[0].length; i++) {
                    confirmedString = confirmedString + "[LAT:" + confirmedList[0] + ", LON:" + confirmedList[1] + "] \n";
                }

                displayMessageToConfirmedList(confirmedString);
            }
        }
    }

    //send grid # for m to search
    void send_to_m (int current_grid_number) {
        toMinion = "AUTOMODE: " + mode + ", GRID: " + current_grid_number;

        //actually send info here
    }

    void setClosestObjectDistance(Robot name) {
        double[] minLoc = new double[2];
        double min = 100000.0;//should be large, not small. I am assuming the position data is positive

        // Calculate distances of objects from robot's current location
        for (int i=0; i<25; i++) {
            if(distanceFormula(name.getRobotLocation(),gpsList[i]) < min && distanceFormula(name.getRobotLocation(),gpsList[i])<100000.0){//the second condition limit the min to be impossible location
                min = distanceFormula(name.getRobotLocation(),gpsList[i]);
                minLoc = gpsList[i];
            }
        }

        name.setDestination(minLoc);
        if(asList(gpsList).contains(minLoc)){

            //assume there is no duplicate location in gpsList
            int index=Arrays.asList(gpsList).indexOf(minLoc);
            searchingList[index]=gpsList[index];
            double[] never_reach_location={100000.0,100000.0};//assume this will never be reached, same purpose with remove the location from array
            gpsList[index]=never_reach_location;
        }

    }

    double distanceFormula(double[] gps1, double[] gps2) {
        return Math.sqrt((gps2[0]-gps1[0])*(gps2[0]-gps1[0]) - (gps2[1]-gps1[1])*(gps2[1]-gps1[1]) );
    }

    double[] getCoords (String str) {
        int find_comma, find_colon;

        //find lat
        find_colon = str.indexOf(':');
        find_comma = str.indexOf(',');
        String first_num = str.substring(find_colon+1, find_comma-1);

        //cut out lat
        str = str.substring(find_comma + 1, str.length());

        //find lon
        find_colon = str.indexOf(':');
        find_comma = str.indexOf(']');
        String sec_num = str.substring(find_colon+1, find_comma-1);


        double[] coords={0,0};
        double d = Double.parseDouble(first_num);
        double d2 = Double.parseDouble(sec_num);
        coords[0] = d;
        coords[1] = d2;
        return coords;
    }

    class Robot {
        Robots name;
        double[] location = new double[2];
        boolean isMannequinFound = false;
        boolean isSearching = false;
        boolean hasLidar = false;
        double[] destination = new double[2];

        public Robot(Robots name, double[] location, boolean hasLidar) {
            this.name = name;
            this.location = location;
            this.hasLidar = hasLidar;
        }

        public Robots getName () {
            return name;
        }

        public double[] getRobotLocation() {
            return location;
        }

        public void setLocation (double[] location) {
            this.location = location;
        }

        public void setDestination (double[] destination) {
            this.destination = destination;
            this.isSearching = true;
        }

        void setStatus(boolean isMannequinFound) {
            this.isMannequinFound = isMannequinFound;

            if (isMannequinFound) {
                this.isSearching = false;
            }
        }

        public boolean getStatus() {
            return this.isMannequinFound;
        }

        public boolean getIsSearching() {
            return this.isSearching;
        }
    }

}