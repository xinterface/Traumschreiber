package de.uni_osnabrueck.ikw.eegdroid;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.content.DialogInterface;
import android.content.pm.ActivityInfo;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.TextView;
import android.widget.Toast;

import com.github.mikephil.charting.charts.LineChart;
import com.github.mikephil.charting.components.Legend;
import com.github.mikephil.charting.components.XAxis;
import com.github.mikephil.charting.components.YAxis;
import com.github.mikephil.charting.data.Entry;
import com.github.mikephil.charting.data.LineData;
import com.github.mikephil.charting.data.LineDataSet;
import com.github.mikephil.charting.highlight.Highlight;
import com.github.mikephil.charting.interfaces.datasets.ILineDataSet;
import com.github.mikephil.charting.listener.OnChartValueSelectedListener;
import com.opencsv.CSVReader;
import com.opencsv.CSVReaderBuilder;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.BasicFileAttributes;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

import de.uni_osnabrueck.ikw.eegdroid.utilities.MyXAxisValueFormatterTime;

/**
 * Activity for display and reproduce EEG Sessions
 */
public class Display extends AppCompatActivity {

    private ArrayList<File> arrayListOfFiles;
    private String[] arrayOfNames;
    private File fileToPlot;
    private LineChart chart;
    private LineDataSet[] lineDataSets;
    private int nChannels = 8;
    private Timer timer;
    private int period = 1;
    private int jump_x = 1;
    private int density = 10;
    private float max_in_X;
    private float position_x = 0;
    private TextView nameTextView;
    private TextView dateTextView;
    private TextView startTextView;
    private TextView finishTextView;
    private BasicFileAttributes attrs;
    private CheckBox chckbx_ch1;
    private CheckBox chckbx_ch2;
    private CheckBox chckbx_ch3;
    private CheckBox chckbx_ch4;
    private CheckBox chckbx_ch5;
    private CheckBox chckbx_ch6;
    private CheckBox chckbx_ch7;
    private CheckBox chckbx_ch8;
    private boolean playing = false;
    private TimerTask updateChart;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_display);
        ManageSessions manager = new ManageSessions();
        arrayListOfFiles = manager.getArrayListOfFiles();

        //Create the list of names for display in dialog
        arrayOfNames = new String[arrayListOfFiles.size()];
        for (int i = 0; i < arrayOfNames.length; i++) {
            arrayOfNames[i] = arrayListOfFiles.get(i).getName();
        }

        OnChartValueSelectedListener ol = new OnChartValueSelectedListener() {
            @Override
            public void onValueSelected(Entry entry, Highlight h) {

                ZonedDateTime point_in_time = attrs.creationTime().toInstant().atZone(ZoneId.systemDefault());
                point_in_time = point_in_time.plusSeconds(Math.round(entry.getX() / 1000));

                Toast.makeText(
                        getApplicationContext(),
                        "Time point: " + Integer.toString(+point_in_time.getHour()) + ":" + Integer.toString(point_in_time.getMinute()) + ":" + Integer.toString(point_in_time.getSecond()),
                        Toast.LENGTH_LONG
                ).show();
            }

            @Override
            public void onNothingSelected() {
            }
        };
        nameTextView = (TextView) findViewById(R.id.name_file);
        dateTextView = (TextView) findViewById(R.id.session_date_file);
        startTextView = (TextView) findViewById(R.id.start_time_file);
        finishTextView = (TextView) findViewById(R.id.finish_time_file);

        chckbx_ch1 = findViewById(R.id.checkBox_ch1_display);
        chckbx_ch2 = findViewById(R.id.checkBox_ch2_display);
        chckbx_ch3 = findViewById(R.id.checkBox_ch3_display);
        chckbx_ch4 = findViewById(R.id.checkBox_ch4_display);
        chckbx_ch5 = findViewById(R.id.checkBox_ch5_display);
        chckbx_ch6 = findViewById(R.id.checkBox_ch6_display);
        chckbx_ch7 = findViewById(R.id.checkBox_ch7_display);
        chckbx_ch8 = findViewById(R.id.checkBox_ch8_display);

        chckbx_ch1.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                if (isChecked) {
                    lineDataSets[0].setVisible(true);
                } else {
                    lineDataSets[0].setVisible(false);
                }
                chart.invalidate();
            }
        });

        chckbx_ch2.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                if (isChecked) {
                    lineDataSets[1].setVisible(true);
                } else {
                    lineDataSets[1].setVisible(false);
                }
                chart.invalidate();
            }
        });

        chckbx_ch3.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                if (isChecked) {
                    lineDataSets[2].setVisible(true);
                } else {
                    lineDataSets[2].setVisible(false);
                }
                chart.invalidate();
            }
        });

        chckbx_ch4.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                if (isChecked) {
                    lineDataSets[3].setVisible(true);
                } else {
                    lineDataSets[3].setVisible(false);
                }
                chart.invalidate();
            }
        });

        chckbx_ch5.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                if (isChecked) {
                    lineDataSets[4].setVisible(true);
                } else {
                    lineDataSets[4].setVisible(false);
                }
                chart.invalidate();
            }
        });

        chckbx_ch6.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                if (isChecked) {
                    lineDataSets[5].setVisible(true);
                } else {
                    lineDataSets[5].setVisible(false);
                }
                chart.invalidate();
            }
        });

        chckbx_ch7.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                if (isChecked) {
                    lineDataSets[6].setVisible(true);
                } else {
                    lineDataSets[6].setVisible(false);
                }
                chart.invalidate();
            }
        });

        chckbx_ch8.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {

                if (isChecked) {
                    lineDataSets[7].setVisible(true);
                } else {
                    lineDataSets[7].setVisible(false);
                }
                chart.invalidate();
            }
        });

        chart = findViewById(R.id.chart);
        chart.setOnChartValueSelectedListener(ol);
        chart.getDescription().setEnabled(false);
        chart.setTouchEnabled(true);
        chart.setDragDecelerationFrictionCoef(0.9f);
        chart.setDragEnabled(true);
        chart.setScaleEnabled(true);
        chart.setDrawGridBackground(false);
        chart.setPinchZoom(true);

        Legend l = chart.getLegend();
        l.setEnabled(false);

        YAxis leftAxis = chart.getAxisLeft();
        leftAxis.setTextColor(Color.GRAY);
        leftAxis.setLabelCount(13, true);
        leftAxis.setDrawGridLines(true);
        leftAxis.setGridColor(Color.WHITE);

        YAxis rightAxis = chart.getAxisRight();
        rightAxis.setEnabled(false);
        rightAxis.setDrawZeroLine(true);

        final XAxis bottomAxis = chart.getXAxis();
        bottomAxis.setLabelCount(5, true);
        bottomAxis.setPosition(XAxis.XAxisPosition.BOTTOM);
        bottomAxis.setGridColor(Color.WHITE);
        bottomAxis.setTextColor(Color.GRAY);

        getSupportActionBar().setDisplayHomeAsUpEnabled(true);

        timer = new Timer();

        // Dialog for choosing the session to plot
        AlertDialog.Builder alert = new AlertDialog.Builder(this)
                .setTitle(R.string.select_recording)
                .setItems(arrayOfNames, new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int which) {

                        fileToPlot = arrayListOfFiles.get(which);
                        loadData(fileToPlot);
                        setData();

                        Path path = arrayListOfFiles.get(which).toPath();

                        try {
                            attrs = Files.readAttributes(path, BasicFileAttributes.class);
                        } catch (IOException ex) {
                            attrs = null;
                        }

                        nameTextView.setText(arrayListOfFiles.get(which).getName());
                        ZonedDateTime creationTime = attrs.creationTime().toInstant().atZone(ZoneId.systemDefault());
                        dateTextView.setText(creationTime.toLocalDate().toString());
                        startTextView.setText(creationTime.toLocalTime().toString());
                        finishTextView.setText(creationTime.toLocalTime().plusSeconds(Math.round(max_in_X / 1000)).toString());

                        bottomAxis.setValueFormatter(new MyXAxisValueFormatterTime(creationTime));

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
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.display_menu, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {

        switch (item.getItemId()) {

            case R.id.play:
                if (!playing) {
                    playing = true;
                    timer = new Timer();
                    updateChart = new UpdateChart();
                    timer.scheduleAtFixedRate(updateChart, 0, period);
                    item.setIcon(R.drawable.ic_pause_white_24dp);
                    return true;

                } else {
                    playing = false;
                    timer.cancel();
                    item.setIcon(R.drawable.ic_play_arrow_white_24dp);
                    return true;
                }

            case R.id.rewind:
                if (!playing) {
                    timer.cancel();
                    timer.purge();
                    chart.moveViewToX(0);
                    position_x = 0;
                }
                return true;

            case R.id.speed_quarter:
                jump_x = 1;
                period = 4;
                chart.invalidate();
                return true;

            case R.id.speed_half:
                jump_x = 1;
                period = 2;
                chart.invalidate();
                return true;

            case R.id.speed_normal:
                jump_x = 1;
                period = 1;
                chart.invalidate();
                return true;

            case R.id.speed_x2:
                jump_x = 2;
                period = 1;
                chart.invalidate();
                return true;

            case R.id.speed_x4:
                jump_x = 4;
                period = 1;
                chart.invalidate();
                return true;

            case R.id.speed_8:
                jump_x = 8;
                period = 1;
                chart.invalidate();
                return true;

            case android.R.id.home:
                onBackPressed();
                return true;

            default:
                return super.onOptionsItemSelected(item);
        }
    }

    // Fills lineDataSets with the content of CSV files
    private void loadData(File file) {

        lineDataSets = new LineDataSet[nChannels];

        try {
            // create csvReader object and skip first 3 Lines
            CSVReader csvReader = new CSVReaderBuilder(new FileReader(file)).withSkipLines(3).build();

            ArrayList<Entry> arrayOfEntry1 = new ArrayList<>();
            ArrayList<Entry> arrayOfEntry2 = new ArrayList<>();
            ArrayList<Entry> arrayOfEntry3 = new ArrayList<>();
            ArrayList<Entry> arrayOfEntry4 = new ArrayList<>();
            ArrayList<Entry> arrayOfEntry5 = new ArrayList<>();
            ArrayList<Entry> arrayOfEntry6 = new ArrayList<>();
            ArrayList<Entry> arrayOfEntry7 = new ArrayList<>();
            ArrayList<Entry> arrayOfEntry8 = new ArrayList<>();


            String[] line;
            int count = 0;
            while ((line = csvReader.readNext()) != null) {

                if (count % density == 0) {
                    arrayOfEntry1.add(new Entry(Float.parseFloat(line[0]), Float.parseFloat(line[1])));
                    arrayOfEntry2.add(new Entry(Float.parseFloat(line[0]), Float.parseFloat(line[2])));
                    arrayOfEntry3.add(new Entry(Float.parseFloat(line[0]), Float.parseFloat(line[3])));
                    arrayOfEntry4.add(new Entry(Float.parseFloat(line[0]), Float.parseFloat(line[4])));
                    arrayOfEntry5.add(new Entry(Float.parseFloat(line[0]), Float.parseFloat(line[5])));
                    arrayOfEntry6.add(new Entry(Float.parseFloat(line[0]), Float.parseFloat(line[6])));
                    arrayOfEntry7.add(new Entry(Float.parseFloat(line[0]), Float.parseFloat(line[7])));
                    arrayOfEntry8.add(new Entry(Float.parseFloat(line[0]), Float.parseFloat(line[8])));
                }
                count++;
            }

            lineDataSets[0] = new LineDataSet(arrayOfEntry1, "Channel_1");
            lineDataSets[1] = new LineDataSet(arrayOfEntry2, "Channel_2");
            lineDataSets[2] = new LineDataSet(arrayOfEntry3, "Channel_3");
            lineDataSets[3] = new LineDataSet(arrayOfEntry4, "Channel_4");
            lineDataSets[4] = new LineDataSet(arrayOfEntry5, "Channel_5");
            lineDataSets[5] = new LineDataSet(arrayOfEntry6, "Channel_6");
            lineDataSets[6] = new LineDataSet(arrayOfEntry7, "Channel_7");
            lineDataSets[7] = new LineDataSet(arrayOfEntry8, "Channel_8");

            csvReader.close();

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void setData() {

        List<ILineDataSet> dataSets = new ArrayList<ILineDataSet>();

        int[] colors = {ContextCompat.getColor(getApplicationContext(), R.color.aqua),
                ContextCompat.getColor(getApplicationContext(), R.color.fuchsia),
                ContextCompat.getColor(getApplicationContext(), R.color.green),
                ContextCompat.getColor(getApplicationContext(), android.R.color.holo_purple),
                ContextCompat.getColor(getApplicationContext(), R.color.orange),
                ContextCompat.getColor(getApplicationContext(), R.color.red),
                ContextCompat.getColor(getApplicationContext(), R.color.yellow),
                ContextCompat.getColor(getApplicationContext(), R.color.black)
        };

        for (int i = 0; i < lineDataSets.length; i++) {
            lineDataSets[i].setAxisDependency(YAxis.AxisDependency.LEFT);
            lineDataSets[i].setColor(colors[i]);
            lineDataSets[i].setValueTextColor(colors[i]);
            lineDataSets[i].setDrawCircles(false);
            lineDataSets[i].setLineWidth(1f);
            lineDataSets[i].setVisible(true);
            dataSets.add(lineDataSets[i]);
        }

        chart.setData(new LineData(dataSets));
        chart.invalidate();
        chart.setVisibleXRangeMaximum(5000); //Shows a maximum of 1 sec per screen
        max_in_X = lineDataSets[0].getXMax();
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        if (timer != null) {
            timer.cancel();
            timer.purge();
        }
    }

    /**
     * Uses a thread to play the session.
     */
    class UpdateChart extends TimerTask {
        public void run() {
            if (position_x > max_in_X) {
                timer.cancel();
                timer.purge();
            }
            chart.moveViewToX(position_x);
            position_x = position_x + jump_x;
        }
    }
}