package de.uni_osnabrueck.ikw.eegdroid;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import android.content.Context;
import android.content.DialogInterface;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.os.Bundle;
import android.view.MenuItem;
import android.webkit.WebView;


public class Tutorial extends AppCompatActivity {

    private WebView webView;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_tutorial);

        getSupportActionBar().setDisplayHomeAsUpEnabled(true);

        webView = (WebView) findViewById(R.id.webViewTutorial);
        webView.getSettings().setJavaScriptEnabled(true);

        if (isNetworkAvailable() == true) {
            webView.loadUrl("https://adrocampos.github.io/EEG-Droid/tutorial.html");
        } else {

            AlertDialog.Builder alert = new AlertDialog.Builder(this)
                    .setTitle(getString(R.string.no_internet))
                    .setMessage(getString(R.string.no_internet_message));

            alert.setPositiveButton(getString(R.string.access_offline), new DialogInterface.OnClickListener() {
                public void onClick(DialogInterface dialog, int which) {
                    webView.loadUrl("file:///android_asset/docs/tutorial.html");
                }
            });
            alert.show();

        }
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

    private boolean isNetworkAvailable() {
        ConnectivityManager connectivityManager
                = (ConnectivityManager) getSystemService(Context.CONNECTIVITY_SERVICE);
        NetworkInfo activeNetworkInfo = connectivityManager.getActiveNetworkInfo();
        return activeNetworkInfo != null && activeNetworkInfo.isConnected();
    }
}
