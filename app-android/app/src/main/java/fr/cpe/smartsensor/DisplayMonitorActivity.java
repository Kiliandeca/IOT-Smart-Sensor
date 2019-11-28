package fr.cpe.smartsensor;

import androidx.annotation.NonNull;
import androidx.appcompat.app.AppCompatActivity;
import androidx.recyclerview.widget.ItemTouchHelper;
import androidx.recyclerview.widget.LinearLayoutManager;
import androidx.recyclerview.widget.RecyclerView;

import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.UnknownHostException;

import java.util.ArrayList;
import java.util.Collections;

import org.json.JSONException;
import org.json.JSONObject;

public class DisplayMonitorActivity extends AppCompatActivity {
    private static final String TAG = MainActivity.class.getSimpleName();

    RecyclerView mRecyclerView;
    MyAdapter mAdapter;
    ItemTouchHelper mItemTouchHelper;
    ArrayList<Model> mArrayList;

    String data, config, ipConf, portConf;
    Button saveButton;

    InetAddress ip;
    int port;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_display_monitor);

        // build RecyclerView and binds its Layout
        mRecyclerView = findViewById(R.id.displayOrderList);
        mRecyclerView.setLayoutManager(new LinearLayoutManager(this));

        // attach ItemTouchHelper to the RecyclerView (used for drag & drop events)
        mItemTouchHelper = new ItemTouchHelper(simpleCallback);
        mItemTouchHelper.attachToRecyclerView(mRecyclerView);

        // get information sent from the MainActivity
        Intent intent = getIntent();
        config = intent.getStringExtra("config");
        portConf = intent.getStringExtra("port");
        ipConf = intent.getStringExtra("ip");

        saveButton = findViewById(R.id.save);
        saveButton.setOnClickListener(onSave());

        // Thread used to fetch the values from the RPi every 10 seconds
        (new Thread() { public void run() {
            while (true) {
                try {
                    sendPacket("getValues()");
                    Thread.sleep(100);
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            try {
                                updateRecyclerViewData();
                            } catch (JSONException e) {
                                e.printStackTrace();
                            }
                        }
                    });

                    Thread.sleep(10000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        } }).start();
    }

    /**
     * Function used to send the new config to the RPi
     * @return OnClickListener
     */
    private View.OnClickListener onSave() {
        return new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sendPacket(config);
            }
        };
    }

    /**
     * Function used to updates the displayed data, launched from the infinite loop in the thread
     * @throws JSONException from JSON/org library
     */
    private void updateRecyclerViewData() throws JSONException {
        ArrayList<Model> models = new ArrayList<>();
        JSONObject obj = new JSONObject(data);

        Model mTemp = new Model();
        mTemp.setCh("T");
        mTemp.setTitle("Temperature");
        mTemp.setDescription(obj.getString("Temp"));
        mTemp.setImg(R.drawable.temperature);

        Model mHumidity = new Model();
        mHumidity.setCh("H");
        mHumidity.setTitle("Humidity");
        mHumidity.setDescription(obj.getString("Humidity"));
        mHumidity.setImg(R.drawable.humidity);

        Model mLuminosity = new Model();
        mLuminosity.setCh("L");
        mLuminosity.setTitle("Luminosity");
        mLuminosity.setDescription(obj.getString("Lux"));
        mLuminosity.setImg(R.drawable.luminosity);

        for (int i=0; i<config.length(); i++) {
            switch (config.substring(i, i+1)) {
                case "T":
                    models.add(mTemp);
                    break;
                case "H":
                    models.add(mHumidity);
                    break;
                case "L":
                    models.add(mLuminosity);
                    break;
            }
        }

        mArrayList = models;
        mAdapter = new MyAdapter(this, mArrayList);
        mRecyclerView.setAdapter(mAdapter);
    }

    ItemTouchHelper.SimpleCallback simpleCallback = new ItemTouchHelper.SimpleCallback(ItemTouchHelper.UP | ItemTouchHelper.DOWN | ItemTouchHelper.START | ItemTouchHelper.END, 0) {
        @Override
        public boolean onMove(@NonNull RecyclerView recyclerView, @NonNull RecyclerView.ViewHolder viewHolder, @NonNull RecyclerView.ViewHolder target) {
            int fromPosition = viewHolder.getAdapterPosition();
            int toPosition = target.getAdapterPosition();

            Collections.swap(mArrayList, fromPosition, toPosition);
            recyclerView.getAdapter().notifyItemMoved(fromPosition, toPosition);

            // builds string from the new items order and saves it as the current config
            StringBuilder newOrder = new StringBuilder();
            for (Model m: mArrayList) {
                newOrder.append(m.getCh());
            }

            config=newOrder.toString();

            return false;
        }

        @Override
        public void onSwiped(@NonNull RecyclerView.ViewHolder viewHolder, int direction) {
        }
    };

    /**
     * Tool function used to creates a Thread and send an UDP packet to the selected ip and port
     * @param message string sent to the server
     */
    public void sendPacket(final String message) {
        (new Thread() { public void run() {
            try {
                // gets server info from config
                ip = InetAddress.getByName(ipConf);
                port = Integer.parseInt(portConf);

                // > Creates socket and datagram and sends it
                DatagramSocket clientSocket;
                clientSocket = new DatagramSocket();

                byte[] sendData;
                sendData = message.getBytes();

                DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, ip, port);
                clientSocket.send(sendPacket);
                Log.d(TAG, "Packet sent: " + message);
                // < Creates socket and datagram and sends it

                // > Waits for response and stores it
                byte[] receiveData = new byte[1024];
                DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
                clientSocket.receive(receivePacket);
                clientSocket.close();

                String lText = new String(receivePacket.getData()).trim();
                Log.d(TAG, "Packet received: " + lText);
                // < Waits for response and stores it

                data = lText;

            } catch (Throwable e) {
                e.printStackTrace();
            }
        } }).start();
    }
}
