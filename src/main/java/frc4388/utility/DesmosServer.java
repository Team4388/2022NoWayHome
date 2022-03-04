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

/** 
 * A http server that allows the robot to communicate with Desmos Graphing Calculator
 * 
 * @author Daniel McGrath
 * */
public class DesmosServer extends Thread {
    private static HashMap<String, String> desmosVariables = new HashMap<>();
    private static HashMap<String, String> readVariables = new HashMap<>();

    private static boolean running = false;
    
    private int activePort;
    
    /** 
     * Creates DesmosServer running on port
     * <p>
     * Use this for cases when the robot is using the default port
     * 
     * @param port The port the server will run on
     * */
    public DesmosServer(int port) {
    	activePort = port;
    }
    
    /** 
     * Creates DesmosServer running on port 5500
     * */
    public DesmosServer() {
    	activePort = 5500;
    }

    @Override
    public void run() {
        try {
            runServer(activePort);
        } catch(Exception err) {
            err.printStackTrace();
        }
    }

    /** 
     * Runs server on port
     * 
     * @param port The port the server runs on
     * @throws IOException
     * */
    public void runServer(int port) throws IOException {
        ServerSocket serverSocket = new ServerSocket(port);
        running = true;

        while(true) {
            Socket client = serverSocket.accept();
            handleClient(client);
        }
    }

    /**
     * Handles client requests
     * 
     * @param The client connection
     * @throws IOException
     * */
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

    /**
     * Sends JSON response
     * 
     * @param The client connection
     * @throws IOException
     * */
    public void sendResponse(Socket client) throws IOException {
        OutputStream clientOutput = client.getOutputStream();
        
        clientOutput.write(getJSONOutput().getBytes());
        clientOutput.flush();
        clientOutput.close();
    }

    /**
     * Produces JSON output containing Desmos output.
     * 
     * @return JSON string to be read by Desmos client
     * */
    public static String getJSONOutput() {
        String json = "[";

        if(!desmosVariables.isEmpty()) {
	        for(String key : desmosVariables.keySet()) {
	            json += "{"
	                + "\"name\":\"" + key + "\","
	                + "\"value\":\"" + desmosVariables.get(key) + "\""
	                + "},";
	        }
	
	        json = json.substring(0, json.length()-1); // remove comma at the end
        }
        
        json += "]";

        return json;
    }

    public static boolean isRunning() {
        return running;
    }
    
    // ---------------------------------------------------------------------

    public static void putInteger(String name, Integer value) {        
        desmosVariables.put(name, value.toString());
    }

    public static void putDecimal(String name, Double value) {
        desmosVariables.put(name, value.toString());
    }

    public static void putPoint(String name, Point point) {
        desmosVariables.put(name, "(" + point.x + "," + point.y + ")");
    }
    
    public static void putIntegerArray(String name, int... arr) {
    	desmosVariables.put(name, Arrays.toString(arr).replace(" ", ""));
    }
    
    public static void putDoubleArray(String name, double... arr) {
    	desmosVariables.put(name, Arrays.toString(arr).replace(" ", ""));
    }
    
    // ---------------------------------------------------------------------

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
