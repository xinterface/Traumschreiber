package de.uni_osnabrueck.ikw.eegdroid;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.ShareActionProvider;
import androidx.core.view.GravityCompat;
import androidx.core.view.MenuItemCompat;
import androidx.drawerlayout.widget.DrawerLayout;

import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

public class Settings extends AppCompatActivity {

    private String saveDir;
    private String username;
    private String userID;
    private String IP;
    private String port;
    private EditText editText_saveDir;
    private EditText editText_username;
    private EditText editText_userID;
    private EditText editText_IP;
    private EditText editText_port;
    private Button applyChangesButton;
    private SharedPreferences sharedPreferences;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_settings);

        //Activates Back button in Menu
        getSupportActionBar().setDisplayHomeAsUpEnabled(true);

        //Retrieve configuration from sharedPreferences
        sharedPreferences = getSharedPreferences("userPreferences", MODE_PRIVATE);
        saveDir = sharedPreferences.getString("saveDir", getResources().getString(R.string.default_folder));
        username = sharedPreferences.getString("username", getResources().getString(R.string.default_username));
        userID = sharedPreferences.getString("userID", getResources().getString(R.string.default_userID));
        IP = sharedPreferences.getString("IP", "192.168.1.125");
        port = sharedPreferences.getString("port", "65432");

        editText_saveDir = (EditText) findViewById(R.id.editText_saveDir);
        editText_username = (EditText) findViewById(R.id.editText_username);
        editText_userID = (EditText) findViewById(R.id.editText_userID);
        editText_IP = (EditText) findViewById(R.id.editText_IP);
        editText_port = (EditText) findViewById(R.id.editText_port);
        applyChangesButton = (Button) findViewById(R.id.settings_apply_changes);

        editText_saveDir.setText(saveDir);
        editText_username.setText(username);
        editText_userID.setText(userID);
        editText_IP.setText(IP);
        editText_port.setText(port);

        //Button to apply changes introduced in EditText
        applyChangesButton.setOnClickListener(new View.OnClickListener() {
            public void onClick(View v) {

                //Write the new data in sharedPreferences
                SharedPreferences.Editor editor = sharedPreferences.edit();
                editor.putString("saveDir", editText_saveDir.getText().toString());
                editor.putString("username", editText_username.getText().toString());
                editor.putString("userID", editText_userID.getText().toString());
                editor.putString("IP", editText_IP.getText().toString());
                editor.putString("port", editText_port.getText().toString());
                editor.apply();

                //Notifies the user
                Toast.makeText(getApplicationContext(), R.string.settings_saved, Toast.LENGTH_LONG).show();

                //Restart the App
                Intent restartIntent = getBaseContext().getPackageManager()
                        .getLaunchIntentForPackage(getBaseContext().getPackageName());
                restartIntent.addFlags(Intent.FLAG_ACTIVITY_CLEAR_TOP);
                startActivity(restartIntent);
            }
        });
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.settings_menu, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {

        switch (item.getItemId()) {

            case android.R.id.home:
                onBackPressed();
                return true;

            case R.id.restore_default_settings:
                //Handles the Dialog to confirm default restore
                AlertDialog.Builder alert = new AlertDialog.Builder(this)
                        .setTitle("Restore confirmation")
                        .setMessage("Do you want to restore the default settings?");
                alert.setPositiveButton(android.R.string.yes, new DialogInterface.OnClickListener() {

                    public void onClick(DialogInterface dialog, int which) {
                        editText_saveDir.setText(getResources().getString(R.string.default_folder));
                        editText_username.setText(getResources().getString(R.string.default_username));
                        editText_userID.setText(getResources().getString(R.string.default_userID));
                        editText_IP.setText(getResources().getString(R.string.default_IP));
                        editText_port.setText(getResources().getString(R.string.default_port));
                    }
                });
                alert.setNegativeButton(android.R.string.cancel, new DialogInterface.OnClickListener() {
                    public void onClick(DialogInterface dialog, int which) {
                        // close dialog
                        dialog.cancel();
                    }
                });
                alert.show();
                return true;

            default:
                return super.onOptionsItemSelected(item);
        }
    }

}
