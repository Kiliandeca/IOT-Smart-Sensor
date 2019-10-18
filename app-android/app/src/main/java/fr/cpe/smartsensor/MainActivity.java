package fr.cpe.smartsensor;

import androidx.appcompat.app.AppCompatActivity;

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
    Button connectBtn;
    InetAddress ip;
    int port;
    private static final String TAG = "MyActivity";
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        connectBtn = findViewById(R.id.connect);
        connectBtn.setOnClickListener(onConnect());
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
