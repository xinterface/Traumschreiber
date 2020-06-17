package de.uni_osnabrueck.ikw.eegdroid.utilities;

import android.content.Context;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.LinearLayout;
import android.widget.TextView;

import androidx.recyclerview.widget.RecyclerView;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.attribute.BasicFileAttributes;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.ArrayList;


import de.uni_osnabrueck.ikw.eegdroid.R;


public class SessionAdapter extends RecyclerView.Adapter<SessionAdapter.PlanetViewHolder> {

    ArrayList<File> arrayListFiles;

    public SessionAdapter(ArrayList<File> arrayListFiles, Context context) {
        this.arrayListFiles = arrayListFiles;
    }

    private int selectedPos = RecyclerView.NO_POSITION;

    @Override
    public SessionAdapter.PlanetViewHolder onCreateViewHolder(ViewGroup parent, int viewType) {
        View v = LayoutInflater.from(parent.getContext()).inflate(R.layout.session_row, parent, false);
        PlanetViewHolder viewHolder = new PlanetViewHolder(v);
        return viewHolder;
    }

    @Override
    public void onBindViewHolder(SessionAdapter.PlanetViewHolder holder, final int position) {

        BasicFileAttributes attrs;
        Path path = arrayListFiles.get(position).toPath();

        try {
            attrs = Files.readAttributes(path, BasicFileAttributes.class);
        } catch (IOException ex) {
            attrs = null;
        }

        holder.name.setText(arrayListFiles.get(position).getName());
        float kbs = attrs.size() / 1000;
        holder.kbs.setText(Float.toString(kbs));

        ZonedDateTime creationTime = attrs.creationTime().toInstant().atZone(ZoneId.systemDefault());
        holder.date.setText(creationTime.toLocalDate().toString());
        holder.hour.setText(creationTime.toLocalTime().toString());

        //This handles the selection of an item in the list
        holder.itemView.setSelected(selectedPos == position);

        holder.linearLayout.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Log.d("Adapter.getSelectedPos() ", Integer.toString(position));
                notifyItemChanged(selectedPos);
                selectedPos = position;
                notifyItemChanged(selectedPos);
            }
        });

    }

    @Override
    public int getItemCount() {
        return arrayListFiles.size();
    }

    public int getSelectedPos() {
        return selectedPos;
    }

    public void resetSelectedPos() {
        selectedPos = RecyclerView.NO_POSITION;
        notifyDataSetChanged();
    }

    public static class PlanetViewHolder extends RecyclerView.ViewHolder {
        protected TextView name;
        protected TextView kbs;
        protected TextView date;
        protected TextView hour;

        public LinearLayout linearLayout;

        public PlanetViewHolder(View itemView) {
            super(itemView);
            name = (TextView) itemView.findViewById(R.id.text_id);
            kbs = (TextView) itemView.findViewById(R.id.session_kbs);
            date = (TextView) itemView.findViewById(R.id.session_date);
            hour = (TextView) itemView.findViewById(R.id.session_hour);
            linearLayout = (LinearLayout) itemView.findViewById(R.id.linear_layout_session_row);
        }
    }


}
