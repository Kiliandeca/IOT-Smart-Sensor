package fr.cpe.smartsensor;

import androidx.appcompat.app.AppCompatActivity;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.UnknownHostException;



public class MainActivity extends AppCompatActivity {
    private static final String TAG = "MyActivity";
    public static final String MSG = "new activity !";

    Button connectBtn, monitorBtn;
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
    }

    private View.OnClickListener onConnect(){
        return new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                String ipAsString = ((EditText)findViewById(R.id.ipForm)).getText().toString();
                try {
                    ip = InetAddress.getByName(ipAsString);
                } catch (UnknownHostException e) {
                    e.printStackTrace();
                }

                String portAsString = ((EditText)findViewById(R.id.portForm)).getText().toString();
                port = Integer.parseInt(portAsString);

                sendPacket("test");
            }
        };
    }

    private View.OnClickListener displayMonitor(final Activity activity){
        return new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                Intent intent = new Intent(activity, DisplayMonitorActivity.class);
                intent.putExtra(MSG, "Dashboard");
                startActivity(intent);
            }
        };
    }

    public void sendPacket(final String message) {
        (new Thread() { public void run() {
            try {
                DatagramSocket UDPSocket = new DatagramSocket();
                byte[] data = message.getBytes();

                DatagramPacket packetCadoNoel = new DatagramPacket(data, data.length, ip, port);
                UDPSocket.send(packetCadoNoel);

            } catch (Exception e) {
                System.out.println(e);
            }
        }}).start();
    }
}
