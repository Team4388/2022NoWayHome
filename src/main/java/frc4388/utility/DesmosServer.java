package frc4388.utility;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.HashMap;

import org.opencv.core.Point;

public class DesmosServer extends Thread {
    private static HashMap<String, String> desmosVariables = new HashMap<>();
    private static HashMap<String, String> readVariables = new HashMap<>();

    private static boolean running = false;

    @Override
    public void run() {
        try {
            runServer(5500);
        } catch(Exception err) {
            err.printStackTrace();
        }
    }

    public void runServer(int port) throws IOException {
        ServerSocket serverSocket = new ServerSocket(port);
        running = true;

        while(true) {
            Socket client = serverSocket.accept();
            handleClient(client);
        }
    }

    public void handleClient(Socket client) throws IOException {
        InputStreamReader clientStream = new InputStreamReader(client.getInputStream());
        BufferedReader bufferedReader = new BufferedReader(clientStream);

        ArrayList<String> requestLines = new ArrayList<>();

        while(bufferedReader.ready()) {
            String read = bufferedReader.readLine();
            requestLines.add(read + "\r\n");
        }

        sendResponse(client);
    }

    public void sendResponse(Socket client) throws IOException {
        OutputStream clientOutput = client.getOutputStream();
        
        clientOutput.write(getJSONOutput().getBytes());
        clientOutput.flush();
        clientOutput.close();
    }

    public static String getJSONOutput() {
        String json = "[";

        for(String key : desmosVariables.keySet()) {
            json += "{"
                + "\"name\":" + key
                + "\"value\":" + desmosVariables.get(key)
                + "},";
        }

        json = json.substring(0, json.length()-1); // remove comma at the end
        json += "]";

        return json;
    }

    public static boolean isRunning() {
        return running;
    }

    public static void putInteger(String name, Integer value) {        
        desmosVariables.put(name, value.toString());
    }

    public static void putDecimal(String name, Double value) {
        desmosVariables.put(name, value.toString());
    }

    public static void putPoint(String name, Point point) {
        desmosVariables.put(name, "(" + point.x + "," + point.y + ")");
    }

    public static int readInteger(String name) {
        return Integer.parseInt(readVariables.get(name));
    }

    public static double readDouble(String name) {
        return Double.parseDouble(readVariables.get(name));
    }

    public static Point readPoint(String name) {
        Point point = new Point();

        String pointStr = readVariables.get(name);
        point.x = Double.parseDouble(pointStr.split(",")[0]);
        point.x = Double.parseDouble(pointStr.split(",")[1]);

        return point;
    }
}
