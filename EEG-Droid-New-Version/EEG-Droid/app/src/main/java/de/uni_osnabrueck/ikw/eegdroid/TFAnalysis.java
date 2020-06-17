package de.uni_osnabrueck.ikw.eegdroid;


import android.content.DialogInterface;
import android.content.pm.ActivityInfo;
import android.graphics.Bitmap;
import android.os.Environment;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.ViewTreeObserver;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ImageView;
import android.widget.LinearLayout;
import android.widget.Spinner;
import android.widget.TableLayout;
import android.widget.Toast;

import org.apache.commons.math3.complex.Complex;
import org.apache.commons.math3.linear.MatrixUtils;

import de.uni_osnabrueck.ikw.eegdroid.utilities.MyXAxisValueFormatterTime;
import de.uni_osnabrueck.ikw.eegdroid.utilities.Utilities;
import de.uni_osnabrueck.ikw.eegdroid.utilities.CustomFFT;
import de.uni_osnabrueck.ikw.eegdroid.utilities.FFTWWrapper;
import de.uni_osnabrueck.ikw.eegdroid.utilities.ZoomableImageView;

import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.data.LineDataSet;
import com.opencsv.CSVReader;
import com.opencsv.CSVReaderBuilder;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.BasicFileAttributes;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TFAnalysis extends AppCompatActivity {

    //ZoomableImageView bmpView;
    private ArrayList<File> arrayListOfFiles;
    private Spinner widthSpinner;
    private Spinner channelSpinner;
    private Spinner overlapSpinner;
    private ImageView bmpView;
    private static final String[] channels = {"1", "2", "3", "4", "5", "6" ,"7", "8"};
    private static final String[] widths = {"1", "2", "3", "4", "5"};
    private static final String[] overlaps = {"0", "1/4", "1/3", "1/2"};
    private static int FS = 225;
    public static int WIDTH = 2*FS;
    public static int CHANNEL = 0;
    public static int OVERLAP = WIDTH/2;
    public double[][] eegData;
    public static int MAXHERTZ = 60;
    public static String WINDOW = "hanning";
    private static int MAX_CHART_HEIGHT;
    private static int MAX_CHART_WIDTH;
    private int density = 10;
    private String[] arrayOfNames;



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_tfanalysis);
        bmpView = findViewById(R.id.spectogram);
        //bmpView = (ZoomableImageView)findViewById(R.id.spectogram);

        ManageSessions manager = new ManageSessions();
        arrayListOfFiles = manager.getArrayListOfFiles();

        //Create the list of names for display in dialog
        arrayOfNames = new String[arrayListOfFiles.size()];
        for (int i = 0; i < arrayOfNames.length; i++) {
            arrayOfNames[i] = arrayListOfFiles.get(i).getName();
        }

        // GENERATE DUMMY DATA
        //double[] sinesData = generateDummyData();

        //LOAD REAL DATA

        //Dialog for choosing the session to plot
        AlertDialog.Builder alert = new AlertDialog.Builder(this)
                .setTitle(R.string.select_tf_analysis)
                .setItems(arrayOfNames, new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int which) {

                        loadData(arrayListOfFiles.get(which));


                            final LinearLayout parent = (LinearLayout) bmpView.getParent();
                            parent.getViewTreeObserver().addOnGlobalLayoutListener(new ViewTreeObserver.OnGlobalLayoutListener() {
                                @Override
                                public void onGlobalLayout() {

                                    try {
                                        parent.getViewTreeObserver().removeOnGlobalLayoutListener(this);
                                        MAX_CHART_HEIGHT = parent.getWidth();
                                        MAX_CHART_WIDTH = parent.getHeight();//height is ready
                                        //Bitmap spectogram = Bitmap.createBitmap(0, 0, Bitmap.Config.ARGB_8888);

                                        Bitmap spectogram = Utilities.getSpectrogramBitmap(eegData[CHANNEL], FS, WIDTH, OVERLAP,
                                                WINDOW, true, MAXHERTZ, true);


                                        int width = spectogram.getWidth();
                                        int height = spectogram.getHeight();
                                        float wScale = (float) MAX_CHART_WIDTH / width;
                                        float hScale = (float) MAX_CHART_HEIGHT / 2 / height;
                                        hScale = Math.min(wScale, hScale);
                                        spectogram = Bitmap.createScaledBitmap(spectogram, (int) (wScale * width), (int) (hScale * height), false);


                                        int test = eegData[CHANNEL].length / FS;
                                        Log.d("zero?", Integer.toString(test));
                                        // get real spectogram
                                        Bitmap chart = Utilities.addAxisAndLabels(spectogram, MAXHERTZ, eegData[CHANNEL].length / FS);

                                        bmpView.setImageBitmap(chart);

                                    } catch (Exception e) {

                                        finish();
                                        Toast.makeText(getApplicationContext(), R.string.warning_too_short, Toast.LENGTH_LONG).show();
                                        Log.d("EXCEPTION", "Dividing by zero");


                                    }

                                }
                            });

                        setupOverlapSpinner();
                        setupWidthSpinner();
                        setupChannelSpinner();
                        getSupportActionBar().setDisplayHomeAsUpEnabled(true);
                        dialog.dismiss();
                        setRequestedOrientation(ActivityInfo.SCREEN_ORIENTATION_LANDSCAPE);


                    }
                }).setOnCancelListener(new DialogInterface.OnCancelListener() {
                    @Override
                    public void onCancel(DialogInterface dialog) {
                        finish();
                    }
                });
        alert.show();



    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case android.R.id.home:
                onBackPressed();
                return true;
            default:
                return super.onOptionsItemSelected(item);
        }
    }

    private void generateSpectogram(){
        Bitmap spectogram = Utilities.getSpectrogramBitmap(eegData[CHANNEL], FS, WIDTH, OVERLAP,
                WINDOW, true, MAXHERTZ, true);

        int width = spectogram.getWidth();
        int height = spectogram.getHeight();
        float wScale = (float) MAX_CHART_WIDTH/width;
        float hScale = (float) MAX_CHART_HEIGHT/2/height;
        hScale = Math.min(wScale,hScale);
        spectogram = Bitmap.createScaledBitmap(spectogram, (int) (wScale*width), (int) (hScale*height), false);

        // get real spectogram
        Bitmap chart = Utilities.addAxisAndLabels(spectogram, MAXHERTZ, eegData[CHANNEL].length/FS);

        bmpView.setImageBitmap(chart);
    }

    private void setupOverlapSpinner(){
        overlapSpinner = (Spinner) findViewById(R.id.overlap_dropdown);
        ArrayAdapter<String> adapter = new ArrayAdapter<String>(this,
                android.R.layout.simple_spinner_item, overlaps);

        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        overlapSpinner.setAdapter(adapter);

        overlapSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view,
                                       int position, long id) {
                int newOVERLAP = OVERLAP;
                switch((String) parent.getItemAtPosition(position)){
                    case "1/2":
                        newOVERLAP = WIDTH/2;
                        break;
                    case "1/3":
                        newOVERLAP = WIDTH/3;
                        break;
                    case "1/4":
                        newOVERLAP = WIDTH/4;
                        break;
                    case "0":
                        newOVERLAP = 0;
                        break;

                }
                if(newOVERLAP!=OVERLAP){
                    OVERLAP = newOVERLAP;
                    generateSpectogram();
                    Log.v("overlap", (String) parent.getItemAtPosition(position));
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                // TODO Auto-generated method stub
            }
        });

        int spinnerPosition = adapter.getPosition("1/2");
        overlapSpinner.setSelection(spinnerPosition);
    }

    private void setupWidthSpinner(){
        widthSpinner = (Spinner) findViewById(R.id.width_dropdown);
        ArrayAdapter<String> adapter = new ArrayAdapter<String>(this,
                android.R.layout.simple_spinner_item, widths);

        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        widthSpinner.setAdapter(adapter);

        widthSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view,
                                       int position, long id) {

                int newWIDTH = Integer.parseInt((String) parent.getItemAtPosition(position)) * FS;
                if(WIDTH!=newWIDTH){
                    double overlap = (double)OVERLAP/WIDTH;
                    WIDTH = newWIDTH;
                    OVERLAP = (int)(WIDTH*overlap);
                    generateSpectogram();
                    Log.v("width", (String) parent.getItemAtPosition(position));
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                // TODO Auto-generated method stub
            }
        });

        int spinnerPosition = adapter.getPosition("2");
        widthSpinner.setSelection(spinnerPosition);
    }

    private void setupChannelSpinner(){
        channelSpinner = (Spinner) findViewById(R.id.channel_dropdown);
        ArrayAdapter<String> adapter = new ArrayAdapter<String>(this,
                android.R.layout.simple_spinner_item, channels);

        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        channelSpinner.setAdapter(adapter);

        channelSpinner.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view,
                                       int position, long id) {
                int newChannel = Integer.parseInt((String) parent.getItemAtPosition(position)) - 1;
                if(CHANNEL!=newChannel){
                    CHANNEL = newChannel;
                    generateSpectogram();
                    Log.v("channel", (String) parent.getItemAtPosition(position));
                }
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                // TODO Auto-generated method stub
            }
        });

        int spinnerPosition = adapter.getPosition("1");
        channelSpinner.setSelection(spinnerPosition);
    }

    //Fills lineDataSets with the content of CSV files
    private void loadData(File file) {

        try {
            // create csvReader object and skip first 3 Lines
            CSVReader csvReader = new CSVReaderBuilder(new FileReader(file)).withSkipLines(3).build();

            List<double[]> list = new ArrayList<>();
            String[] line;
            while ((line = csvReader.readNext()) != null) {
                double[] parsed = new double[line.length - 2];
                //Ignore the first and last entry of every list
                for (int i = 1; i < line.length - 1; i++) {
                    parsed[i - 1] = Double.parseDouble(line[i]);
                }
                list.add(parsed);
            }

            //Convert the list in double[][] and transpose the data
            eegData = MatrixUtils.createRealMatrix(list.toArray(new double[][]{})).transpose().getData();
            csvReader.close();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

}