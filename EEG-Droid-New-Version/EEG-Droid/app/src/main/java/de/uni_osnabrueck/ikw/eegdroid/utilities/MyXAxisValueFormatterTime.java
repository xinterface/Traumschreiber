package de.uni_osnabrueck.ikw.eegdroid.utilities;

import com.github.mikephil.charting.components.AxisBase;
import com.github.mikephil.charting.formatter.IAxisValueFormatter;

import java.time.LocalTime;
import java.time.ZonedDateTime;

public class MyXAxisValueFormatterTime implements IAxisValueFormatter {

    private ZonedDateTime time;

    public MyXAxisValueFormatterTime(ZonedDateTime creationTime) {
        time = creationTime;
    }

    @Override
    public String getFormattedValue(float value, AxisBase axis) {
        // Value represents the position of the label on the axis (x or y)
        LocalTime localTime = time.toLocalTime().plusSeconds(Math.round(value / 1000));
        return localTime.toString();
    }

}