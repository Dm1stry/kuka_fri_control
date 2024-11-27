package application;


import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;

public class RemoteConnection implements Runnable {
    private int port;
    private MessageListener listener;
    private Output output;
    private boolean running = false;

    private ServerSocket serverSocket;
    private Socket clientSocket;

    public RemoteConnection(MessageListener listener, Output output) 
    {
        this(listener, output, 30001);
    }

    public RemoteConnection(MessageListener listener, Output output, int port) 
    {
        this.port = port;
        this.listener = listener;
        this.output = output;
    }

    @Override
    public void run() 
    {
        try {
            serverSocket = new ServerSocket(port);
            running = true;
            output.print("TCP Server is listening on port " + port);

            clientSocket = serverSocket.accept(); // Accept a new client
            output.print("Client connected: " + clientSocket.getInetAddress());

            handleClient();
            
        } catch (Exception e) {
            output.printerr("Server error: " + e.getMessage());
        }
    }

    private void handleClient() 
    {
        try {
        	InputStream in = clientSocket.getInputStream();
            byte[] buffer = new byte[1024]; // Buffer for reading data
            int bytesRead;

            while ((bytesRead = in.read(buffer)) != -1 && running) 
            {
                byte[] data = new byte[bytesRead];
                System.arraycopy(buffer, 0, data, 0, bytesRead);

                if (listener != null) 
                {
                    listener.onMessageReceived(data, clientSocket);
                }
            }
        } // try
        catch (Exception e) 
        {
            output.printerr("Error handling client: " + e.getMessage());
        } 
        finally 
        {
            try 
            {
                clientSocket.close();
                output.print("Client disconnected: " + clientSocket.getInetAddress());
            } 
            catch (Exception e) 
            {
                output.printerr("Error closing client socket: " + e.getMessage());
            }
        } // finally
    } // handleClient() 

    public void sendData(byte[] data) 
    {
        try {
        	OutputStream outputClientDataStream = clientSocket.getOutputStream();
            outputClientDataStream.write(data);
            outputClientDataStream.flush(); // Ensure all data is sent
        } catch (Exception e) {
            output.printerr("Error sending data: " + e.getMessage());
        }
    }

    public void stop() {
        running = false;
    }
}
