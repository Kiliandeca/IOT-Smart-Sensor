package fr.cpe.smartsensor;

import androidx.appcompat.app.AppCompatActivity;
import android.app.Activity;
import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.UnknownHostException;


public class MainActivity extends AppCompatActivity {
    private static final String TAG = MainActivity.class.getSimpleName();

    String config;
    Button connectBtn, monitorBtn;
    TextView feedback;
    InetAddress ip;
    int port;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        connectBtn = findViewById(R.id.connect);
        connectBtn.setOnClickListener(onConnect());

        monitorBtn = findViewById(R.id.monitor);
        monitorBtn.setOnClickListener(displayMonitor(this));

        feedback = findViewById(R.id.feedback);
    }

    /**
     * Method used to check the connection info by sending a "hello" packet
     * @return OnClickListener
     */
    private View.OnClickListener onConnect() {
        return new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                String ipAsString = ((EditText)findViewById(R.id.ipForm)).getText().toString();
                try {
                    ip = InetAddress.getByName(ipAsString);
                    String portAsString = ((EditText)findViewById(R.id.portForm)).getText().toString();
                    port = Integer.parseInt(portAsString);
                    sendPacket("hello");
                } catch (UnknownHostException e) {
                    e.printStackTrace();
                }
            }
        };
    }

    /**
     * Method used to get the saved config on the RPi and start the new DisplayMonitorActivity
     * @param activity mainActivity
     * @return OnClickListener
     */
    private View.OnClickListener displayMonitor(final Activity activity){
        return new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                sendPacket("getConfig()");
                try {
                    Thread.sleep(100);
                    Intent intent = new Intent(activity, DisplayMonitorActivity.class);

                    // Saves info to send them to the new Activity and starts it
                    intent.putExtra("config", config);
                    intent.putExtra("ip", String.valueOf(ip.getHostAddress()));
                    intent.putExtra("port", String.valueOf(port));

                    startActivity(intent);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        };
    }

    /**
     * Tool function used to creates a Thread and send an UDP packet to the selected ip and port
     * @param message string sent to the server
     */
    public void sendPacket(final String message) {
        (new Thread() { public void run() {
            try {
                // > Creates socket and datagram and sends it
                DatagramSocket clientSocket;
                clientSocket = new DatagramSocket();

                byte[] sendData;
                byte[] receiveData = new byte[1024];

                sendData = message.getBytes();

                DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, ip, port);
                clientSocket.send(sendPacket);
                Log.d(TAG, "Packet sent: " + message);
                // < Creates socket and datagram and sends it

                // > Waits for response and stores it
                DatagramPacket receivePacket = new DatagramPacket(receiveData, receiveData.length);
                clientSocket.receive(receivePacket);
                clientSocket.close();

                String lText = new String(receivePacket.getData()).trim();
                Log.d(TAG, "Packet received: " + lText);
                // < Waits for response and stores it

                // Connection feedback check
                if (lText.equals("helloBack")) {
                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            monitorBtn.setVisibility(View.VISIBLE);
                            feedback.setVisibility(View.VISIBLE);
                            feedback.setTextColor(Color.GREEN);
                        }
                    });
                    return;
                }
                config = lText;
            } catch (Throwable e) {
                e.printStackTrace();
            }
        } }).start();
    }
}
