package frc4388.utility;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import org.opencv.core.Point;

/** 
 * A http server that allows the robot to communicate with Desmos Graphing Calculator
 * 
 * @author Daniel McGrath
 * */
public class DesmosServer extends Thread {
    private static HashMap<String, String[]> desmosVariables = new HashMap<>();
    private static HashMap<String, String[]> readVariables = new HashMap<>();

    private static boolean running = false;
    
    public static boolean debug = false;
    
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
    	System.out.println("Initializing DesmosServer on port " + port + "...");
    	
        ServerSocket serverSocket = new ServerSocket(port);
        running = true;
        
        System.out.println("DesmosServer is active!");

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

        ArrayList<String> headers = new ArrayList<>();
        
        if(debug) {
        	System.out.println("debug");
        }

        String header;
        while((header = bufferedReader.readLine()).length() != 0) {
            headers.add(header);
        }
        
        String body = "";
        while(bufferedReader.ready()) {
            body += (char) bufferedReader.read();
        }
        
        readVariables(body);
        sendResponse(client);
    }

    /**
     * Sends JSON response with format
     * <p>
     * [
     *     {"int": "24"},
     *     {"double": "2.4"},
     *     {"point": "(2,4)"},
     *     {"list": "[2,4]"}
     * ]
     * 
     * @param The client connection
     * @throws IOException
     * */
    public void sendResponse(Socket client) throws IOException {
        OutputStream clientOutput = client.getOutputStream();
        
        // Write Headers
        clientOutput.write("HTTP/1.1 200 OK\r\n".getBytes());
        clientOutput.write("Access-Control-Allow-Origin: *\r\n".getBytes());
        clientOutput.write("Keep-Alive: timeout=2, max=100\r\n".getBytes());
        clientOutput.write("Connection: Keep-Alive\r\n".getBytes());
        clientOutput.write("Content-Type: application/json\r\n\r\n".getBytes());
        
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
	            json += "\n\t{"
	                + "\"name\":\"" + key + "\","
                    + "\"type\":\"" + desmosVariables.get(key)[0] + "\","
	                + "\"value\":\"" + desmosVariables.get(key)[1] + "\""
	                + "},";
	        }
	
	        json = json.substring(0, json.length()-1); // remove comma at the end
        }
        
        json += "\n]";

        return json;
    }
    
    /**
     * Interpret client request and update variables
     * 
     * @param requestBody Client request
     */
    public static void readVariables(String requestBody) {
    	for(String variable : requestBody.split("\n")) {
    		if(variable.equals(""))
    			break;
    		
    		String[] readVar = variable.split("\t");
    		readVariables.put(readVar[0], new String[] {readVar[1], readVar[2]});
    	}
    }

    /**
     * Checks if the server is running
     * 
     * @return The server status
     */
    public static boolean isRunning() {
        return running;
    }
    
    // ---------------------------------------------------------------------

    public static void putInteger(String name, Integer value) {        
        desmosVariables.put(name, new String[] {"integer", value.toString()});
    }

    public static void putDouble(String name, Double value) {
        desmosVariables.put(name, new String[] {"double", value.toString()});
    }

    public static void putPoint(String name, Point point) {
        desmosVariables.put(name, new String[] {"point", "(" + point.x + "," + point.y + ")"});
    }
    
    public static void putArray(String name, double... arr) {
    	desmosVariables.put(name, new String[] {"array", Arrays.toString(arr).replace(" ", "")});
    }

    public static void putTable(String name, Object... table) {
        // Check parameters
        for(int i = 0; i < table.length; i += 2)
            if(!(table[i] instanceof String)) { return; }

        for(int i = 1; i < table.length; i += 2)
            if(!(table[i] instanceof double[])) { return; }
        
        String tableStr = "";

        for(int i = 0; i < table.length; i += 2) {
            tableStr += table[i] + ",";
            String values = Arrays.toString((double[]) table[i+1]).replace(" ", "");
            tableStr += values.substring(1, values.length() - 1);
            tableStr += " ";
        }

        tableStr = tableStr.substring(0, tableStr.length()-1); // remove tab at the end

        desmosVariables.put(name, new String[] {"table", tableStr});
    }

    // ---------------------------------------------------------------------

    public static int readInteger(String name) {
        if(!readVariables.containsKey(name) && !readVariables.get(name)[0].equals("integer"))
            return 0;
        
        return Integer.parseInt(readVariables.get(name)[1]);
    }

    public static double readDouble(String name) {
        if(!readVariables.containsKey(name) && !readVariables.get(name)[0].equals("double"))
            return 0;
        
        return Double.parseDouble(readVariables.get(name)[1]);
    }

    public static Point readPoint(String name) {
        Point point = new Point();

        if(!readVariables.containsKey(name) && !readVariables.get(name)[0].equals("point"))
            return point;

        String pointStr = readVariables.get(name)[1];
        point.x = Double.parseDouble(pointStr.split(",")[0]);
        point.x = Double.parseDouble(pointStr.split(",")[1]);

        return point;
    }
    
    public static double[] readArray(String name) {
        if(!readVariables.containsKey(name) && !readVariables.get(name)[0].equals("array"))
            return new double[0];

    	String[] unparsed = readVariables.get(name)[1].split(",");
    	double[] arr = new double[unparsed.length];
    	
    	for(int i = 0; i < arr.length; i++)
    		arr[i] = Integer.parseInt(unparsed[i]);
    	
    	return arr;
    }
}