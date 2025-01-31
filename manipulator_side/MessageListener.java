package application;

import java.net.Socket;

public interface MessageListener {
    void onMessageReceived(byte[] message, Socket clientSocket);
}
