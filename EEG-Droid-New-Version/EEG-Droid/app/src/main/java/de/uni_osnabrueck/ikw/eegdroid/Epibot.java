package de.uni_osnabrueck.ikw.eegdroid;

/*
 * Copyright 2017 IBM Corp. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
 * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 */


import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.VibrationEffect;
import android.os.Vibrator;
import android.text.Editable;
import android.text.TextWatcher;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.widget.ArrayAdapter;
import android.widget.EditText;
import android.widget.ImageButton;
import android.widget.ListView;
import android.widget.Toast;

import androidx.appcompat.app.AlertDialog;
import androidx.appcompat.app.AppCompatActivity;

import com.ibm.watson.developer_cloud.android.library.audio.MicrophoneHelper;
import com.ibm.watson.developer_cloud.android.library.audio.MicrophoneInputStream;
import com.ibm.watson.developer_cloud.android.library.audio.StreamPlayer;
import com.ibm.watson.developer_cloud.android.library.audio.utils.ContentType;
import com.ibm.watson.developer_cloud.conversation.v1.Conversation;
import com.ibm.watson.developer_cloud.conversation.v1.model.Context;
import com.ibm.watson.developer_cloud.conversation.v1.model.DialogRuntimeResponseGeneric;
import com.ibm.watson.developer_cloud.conversation.v1.model.InputData;
import com.ibm.watson.developer_cloud.conversation.v1.model.MessageOptions;
import com.ibm.watson.developer_cloud.conversation.v1.model.MessageRequest;
import com.ibm.watson.developer_cloud.conversation.v1.model.MessageResponse;
import com.ibm.watson.developer_cloud.conversation.v1.model.OutputData;
import com.ibm.watson.developer_cloud.http.ServiceCallback;
import com.ibm.watson.developer_cloud.speech_to_text.v1.SpeechToText;
import com.ibm.watson.developer_cloud.speech_to_text.v1.model.RecognizeOptions;
import com.ibm.watson.developer_cloud.speech_to_text.v1.model.SpeechRecognitionResults;
import com.ibm.watson.developer_cloud.speech_to_text.v1.websocket.BaseRecognizeCallback;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

;

public class Epibot extends AppCompatActivity {

    private final String TAG = "MainActivity";
    private EditText input;
    private ImageButton mic;
    private ImageButton send;
    private SpeechToText speechToText;
    private static Conversation conversationService;
    private StreamPlayer player = new StreamPlayer();
    private MicrophoneHelper microphoneHelper;
    private MicrophoneInputStream capture;
    private boolean listening = false;
    private Handler handler = new Handler();
    public ListView msgView;
    private ArrayAdapter<String> msgList;
    private ArrayList<String[]> messages;
    private MessageAdapter adapter;
    private Context context;
    private int counter_interactions;
    private String username;
    private String botsname;
    private File conversation_file;
    private String conversation_name = "/conversation.tmp";


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_epibot);
        counter_interactions = 0;
        microphoneHelper = new MicrophoneHelper(this);
        speechToText = initSpeechToTextService();
        conversationService = initConversationService();
        final String inputWorkspaceId = getString(R.string.conversation_workspaceId);
        //Msg is null
        msgView = (ListView) findViewById(R.id.listView2);
        msgList = new ArrayAdapter<String>(this, android.R.layout.simple_list_item_1);
        msgView.setAdapter(msgList);
        input = (EditText) findViewById(R.id.input);
        mic = (ImageButton) findViewById(R.id.mic);
        send = (ImageButton) findViewById(R.id.send);
        MessageResponse response = null;
        conversationAPI(String.valueOf(input.getText()), context, inputWorkspaceId);

        conversation_file = new File(getFilesDir().getPath() + conversation_name);

        try {
            FileInputStream fis = new FileInputStream(conversation_file);
            ObjectInputStream ois = new ObjectInputStream(fis);
            messages = (ArrayList<String[]>) ois.readObject();
            ois.close();
        } catch (FileNotFoundException e) {
            messages = new ArrayList<String[]>();
            Log.d("onCreate", e.getMessage());
        } catch (Exception e) {
            messages = new ArrayList<String[]>();
            Log.d("onCreate", e.getMessage());
        }

        adapter = new MessageAdapter(this, R.layout.adapter_view_layout, messages);
        msgView.setAdapter(adapter);
        username = getString(R.string.username);
        botsname = getString(R.string.botsname);


        mic.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                // mic.setEnabled(false);
                if (!listening) {
                    capture = microphoneHelper.getInputStream(true);
                    new Thread(new Runnable() {
                        @Override
                        public void run() {
                            try {
                                speechToText.recognizeUsingWebSocket(getRecognizeOptions(capture), new MicrophoneRecognizeDelegate());
                            } catch (Exception e) {
                                showError(e);
                            }
                        }
                    }).start();
                    listening = true;
                } else {
                    microphoneHelper.closeInputStream();
                    listening = false;
                }
            }
        });


        send.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                //pressing the [Send] button passes the text to the WCS conversation service
                MessageResponse response = null;
                String text = String.valueOf(input.getText());
                input.setText(String.valueOf(""));
                conversationAPI(String.valueOf(text), context, inputWorkspaceId);
            }
        });

        getSupportActionBar().setDisplayHomeAsUpEnabled(true);

        if (!isNetworkAvailable()){
            mic.setEnabled(false);
            send.setEnabled(false);
            input.setEnabled(false);
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        getMenuInflater().inflate(R.menu.epibot_menu, menu);
        return true;
    }

    private void showError(final Exception e) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                //Toast.makeText(Epibot.this, e.getMessage(), Toast.LENGTH_SHORT).show();
                Toast.makeText(Epibot.this, getResources().getText(R.string.epibot_no_internet), Toast.LENGTH_SHORT).show();
                e.printStackTrace();
            }
        });
    }

    private void showMicText(final String text) {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                input.setText(text);
            }
        });
    }

    private void enableMicButton() {
        runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mic.setEnabled(true);
            }
        });
    }

    private SpeechToText initSpeechToTextService() {
        SpeechToText service = new SpeechToText();
        String username = getString(R.string.speech_text_username);
        String password = getString(R.string.speech_text_password);
        service.setUsernameAndPassword(username, password);
        service.setEndPoint(getString(R.string.speech_text_url));
        return service;
    }

    private Conversation initConversationService() {
        Conversation service = new Conversation("2018-07-10");
        String username = getString(R.string.conversation_username);
        String password = getString(R.string.conversation_password);
        service.setUsernameAndPassword(username, password);
        service.setEndPoint(getString(R.string.conversation_url));
        return service;
    }

    private RecognizeOptions getRecognizeOptions(MicrophoneInputStream capture) {
        return new RecognizeOptions.Builder().audio(capture).contentType(ContentType.OPUS.toString())
                .model("en-US_BroadbandModel").inactivityTimeout(2000).build();
    }

    private abstract class EmptyTextWatcher implements TextWatcher {
        private boolean isEmpty = true; // assumes text is initially empty

        @Override
        public void beforeTextChanged(CharSequence s, int start, int count, int after) {
        }

        @Override
        public void onTextChanged(CharSequence s, int start, int before, int count) {
            if (s.length() == 0) {
                isEmpty = true;
                onEmpty(true);
            } else if (isEmpty) {
                isEmpty = false;
                onEmpty(false);
            }
        }

        @Override
        public void afterTextChanged(Editable s) {
        }

        public abstract void onEmpty(boolean empty);
    }

    private class MicrophoneRecognizeDelegate extends BaseRecognizeCallback {

        @Override
        public void onTranscription(SpeechRecognitionResults speechResults) {
            System.out.println(speechResults);
            if (speechResults.getResults() != null && !speechResults.getResults().isEmpty()) {
                String text = speechResults.getResults().get(0).getAlternatives().get(0).getTranscript();
                showMicText(text);
            }
        }

        @Override
        public void onError(Exception e) {
            showError(e);
            enableMicButton();
        }

        @Override
        public void onDisconnected() {
            enableMicButton();
        }
    }


    /**
     * On request permissions result.
     *
     * @param requestCode  the request code
     * @param permissions  the permissions
     * @param grantResults the grant results
     */
    @Override
    public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
        switch (requestCode) {
            case MicrophoneHelper.REQUEST_PERMISSION: {
                if (grantResults.length > 0 && grantResults[0] != PackageManager.PERMISSION_GRANTED) {
                    Toast.makeText(this, "Permission to record audio denied", Toast.LENGTH_SHORT).show();
                }
            }
        }
    }

    /**
     * On activity result.
     *
     * @param requestCode the request code
     * @param resultCode  the result code
     * @param data        the data
     */
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
    }

    public void conversationAPI(String input, Context context, String workspaceId) {

        InputData.Builder inputDataBuilder = new InputData.Builder(input);
        InputData inputData = inputDataBuilder.build();
        MessageRequest newMessage = new MessageRequest();
        newMessage.setInput(inputData);
        newMessage.setContext(context);

        if (input.trim().length() > 0) {
            String[] user_input_null = {username, input, "text"};
            messages.add(user_input_null);
            msgView.setAdapter(adapter);
        }

        MessageOptions messageOptions = new MessageOptions.Builder().workspaceId(workspaceId).messageRequest(newMessage).context(context).input(inputData).build();

        conversationService.message(messageOptions).enqueue(new ServiceCallback<MessageResponse>() {
            @Override
            public void onResponse(MessageResponse response) {
                displayMsg(response);
            }

            @Override
            public void onFailure(Exception e) {
                showError(e);
            }
        });
    }

    public void displayMsg(MessageResponse msg) {
        final MessageResponse mssg = msg;
        OutputData outputData = new OutputData();
        outputData = mssg.getOutput();
        DialogRuntimeResponseGeneric generic = outputData.getGeneric().get(0);


        if (generic.getResponseType().equals("text")) {

//            final String text = outputData.getText().get(0);
//            final String[] textSplit = text.split(Pattern.quote(". "));
//            final List<String> texts = Arrays.asList(textSplit);
//
//            for (final String element : texts) {
//
//                delay(element, 750);
//                vibration(100);
//
//                handler.post(new Runnable() {
//                    @Override
//                    public void run() {
//                        String[] user_msj_link = {botsname, element, "text"};
//                        messages.add(user_msj_link);
//                        msgView.setAdapter(adapter);
//                        context = mssg.getContext();
//                    }
//                });
//            }


            final String text = outputData.getText().get(0);
            vibration(100);
            handler.post(new Runnable() {
                @Override
                public void run() {
                    String[] user_msj_link = {botsname, text, "text"};
                    messages.add(user_msj_link);
                    msgView.setAdapter(adapter);
                    context = mssg.getContext();
                }
            });


        } else if (generic.getResponseType().equals("image")) {

            final String text = generic.getSource();
            //delay("abcdefghiklmnopqrstvwxyz", 750);
            vibration(100);

            handler.post(new Runnable() {
                @Override
                public void run() {
                    String[] user_msj_link = {botsname, text, "image"};
                    messages.add(user_msj_link);
                    msgView.setAdapter(adapter);
                    context = mssg.getContext();
                }
            });

        }


    }

    public void delay(String text, int delayPerLine) {

        int delay = delayPerLine * (text.length() / 25);

        try {
            Thread.sleep(delay);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }

    }

    public void vibration(int duration) {
        if (Build.VERSION.SDK_INT >= 26) {
            ((Vibrator) getSystemService(VIBRATOR_SERVICE)).vibrate(VibrationEffect.createOneShot(duration, VibrationEffect.DEFAULT_AMPLITUDE));
        } else {
            ((Vibrator) getSystemService(VIBRATOR_SERVICE)).vibrate(duration);
        }
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {

        switch (item.getItemId()) {
            case android.R.id.home:
                onBackPressed();
                return true;

            case R.id.clear_conversation:

                //Handles the Dialog to confirm the file delete
                AlertDialog.Builder alert = new AlertDialog.Builder(this)
                        .setTitle(R.string.clear_conversation)
                        .setMessage(getResources().getString(R.string.clear_conversation_message));
                alert.setPositiveButton(android.R.string.yes, new DialogInterface.OnClickListener() {

                    public void onClick(DialogInterface dialog, int which) {
                        messages.clear();
                        msgList.notifyDataSetChanged();
                        adapter.notifyDataSetChanged();
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

    @Override
    protected void onDestroy() {
        super.onDestroy();
        try {
            FileOutputStream fos = new FileOutputStream(conversation_file);
            ObjectOutputStream oos = new ObjectOutputStream(fos);
            oos.writeObject(messages);
            oos.close();
        } catch (FileNotFoundException e) {
            Log.d("onDestroy", e.getMessage());
        } catch (Exception e) {
            Log.d("onDestroy", e.getMessage());

        }
    }

    private boolean isNetworkAvailable() {
        ConnectivityManager connectivityManager
                = (ConnectivityManager) getSystemService(android.content.Context.CONNECTIVITY_SERVICE);
        NetworkInfo activeNetworkInfo = connectivityManager.getActiveNetworkInfo();
        return activeNetworkInfo != null && activeNetworkInfo.isConnected();
    }
}

