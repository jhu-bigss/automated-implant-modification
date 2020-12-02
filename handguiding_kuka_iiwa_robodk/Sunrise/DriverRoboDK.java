/*
RoboDK Driver for KUKA IIWA robots
Copyright 2015-2019 - RoboDK Inc. - https://robodk.com/
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

// If this line is not accepted, comment or replace with the following line (package application)
package com.robodk;
// package application;

import java.net.ServerSocket;
import java.net.Socket;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import java.lang.reflect.Method;

import com.kuka.generated.ioAccess.MediaFlangeIOGroup;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.*;
import static com.kuka.roboticsAPI.motionModel.HRCMotions.*;

import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.Transformation;

import com.kuka.roboticsAPI.motionModel.CIRC;
import com.kuka.roboticsAPI.motionModel.LIN;
import com.kuka.roboticsAPI.motionModel.PTP;

import javax.inject.Inject;

@SuppressWarnings("all")
public class DriverRoboDK extends RoboticsAPIApplication {

    //--------------------------------------------------------
    //------------- HOW TO SETUP --------------------
    // How the RoboDK driver for KUKA IIWA works:
    // 1- Place the apikukaiiwa.py file here:
    //   C:/RoboDK/api/robot/apikukaiiwa.py
    // 2- Set the API driver path to apikukaiiwa
    //   Load this JAVA file in the KUKA IIWA controller
    // 3- Enter the robot IP and port (30000 or as specified by DRIVER_PORT variable)
    //
    //------------- CONFIGURATION SECTION --------------------
    // Set the robot driver port (default is 30000)
    // a different port may be used if the desired port is busy. Check the log
    static final int DRIVER_PORT = 30000;

    // refresh rate to monitor the robot position and display it in RoboDK
    // it is recommended to use a value between 10 to 500 Hz (updates / second)
    static final int REFRESH_RATE = 50;

    // Enter any subprograms that you would like to call given a name
    // In RoboDK, you should provoke a call to the program name (by name, not number)
    class SubPrograms{
        public void SetForceConditionOnce() {
//            String targetName = "";
//            try {
//                targetName = readLine();
//            } catch (IOException e) {
//                e.printStackTrace();
//            }
//            System.out.println("targetName: " + targetName);
//            Frame frame = robot.getCurrentCartesianPosition(robot.getFlange());
//            frame.transform(robot.getFlange(), Transformation.ofTranslation(0, 0, 30));
//            ForceCondition forceCondition = ForceCondition.createSpatialForceCondition(robot.getFlange(), 10);
//            TOOL.move(lin(frame).setCartVelocity(50).breakWhen(forceCondition));
            double threshold;
            try {
                threshold = Double.parseDouble(readLine());
            } catch (IOException e) {
                e.printStackTrace();
                return;
            }
            System.out.println("Set force condition to threshold " + threshold + ". It will be used once.");
            forceConditionSingleUse = ForceCondition.createSpatialForceCondition(TOOL.getDefaultMotionFrame(), threshold);
        }

        public void Program2(){}
    }
    //--------------------------------------------------------
    SubPrograms subprogs = new SubPrograms();

    // robot specific variables
    private LBR robot;
	private Tool TOOL;
	@Inject protected MediaFlangeIOGroup robotIOs;

    // force condition
    ForceCondition forceConditionSingleUse = null;


    // socket read/write variables
    DataInputStream dis = null;
    DataOutputStream dos = null;
    private static ServerSocket server;
    private static Socket client;
    private boolean isDisposed = false;

    static final int MSG_CJNT = 1;
    static final int MSG_SETTOOL = 2;
    static final int MSG_SPEED = 3;
    static final int MSG_ROUNDING = 4;

    static final int MSG_MOVEJ = 10;
    static final int MSG_MOVEL = 11;
    static final int MSG_MOVEC = 12;
    static final int MSG_MOVEL_SEARCH = 13;
    static final int MSG_HAND_GUIDING = 15;

    static final int MSG_POPUP = 20;
    static final int MSG_PAUSE = 21;
    static final int MSG_RUNPROG = 22;
    static final int MSG_SETDO = 23;
    static final int MSG_WAITDI = 24;
    static final int MSG_GETDI = 25;
    static final int MSG_SETAO = 26;
    static final int MSG_GETAI = 27;
    
    static final int MSG_DISCONNECT = 999;

    static final int MSG_MONITOR = 127;
    static final int MSG_ACKNOWLEDGE = 128;

    /*
    public void main(String args[]) throws IOException, ClassNotFoundException{
        run();

    }*/

    public void initialize() {
		robot = getContext().getDeviceFromType(LBR.class);
	}

	@Override
	public void dispose(){
		isDisposed = true;
		//System.out.println("Stopping robot...");
		//if (motionContainer != null) motionContainer.cancel();
		System.out.println("Closing the sockets... ");
		try { server.close(); } catch (Exception e) { }
		try { client.close(); } catch (Exception e) { }
    }

    // Default communication messages
    private int readInt() throws IOException {
        return dis.readInt();
    }

    private void sendInt(int value) throws IOException {
        dos.writeInt(value);;
    }
    private Double readDouble() throws IOException {
        return dis.readDouble();
    }
    private void sendDouble(Double value) throws IOException {
        System.out.println("sendDouble(value=" + value + ")");
        dos.writeDouble(value);;
    }
    private String readLine() throws IOException {
        int c = dis.read();
        if (c == -1){// if EOF
            return null;
        }
        StringBuilder builder = new StringBuilder("");
        while (c != -1 && c != 0){ // Check if new line or EOF
            builder.append((char) c);
            c = dis.read();
        }
        return builder.toString();
    }
    private void sendLine(String s) throws IOException {
        dos.writeBytes(s);
        dos.write(0);
    }
    private List<Double> readArray() throws IOException, Exception {
        return readArray(-1);
    }
    /*
      @param nval_required: ensure that at least this number of values are received; -1 to disable assertion
     */
    private List<Double> readArray(int nval_required) throws IOException, Exception {
        List<Double> list = new ArrayList<Double>();
        int nval_received = dis.readInt();
        if (nval_required != -1 && nval_received < nval_required){
            throw new Exception("The robot didn't receive the minimum number of values required");
        }
        for (int i=0; i<nval_received; i++){
            list.add(dis.readDouble());
        }
        return list;
    }
    private void sendArray(List<Double> values) throws IOException {
        System.out.println("sendArray(values=" + values + ")");
        System.out.println("writeInt: " + values.size());
        dos.writeInt(values.size());
        for (int i=0; i<values.size(); i++){
            dos.writeDouble(values.get(i));
        }
    }

    /// Monitor the robot while it moves
    private void waitMove() throws IOException {
    	while (!robot.isReadyToMove()){
    		List<Double> jntsmove = new ArrayList<Double>();
    		sendInt(MSG_MONITOR);
            double jnts[] = robot.getCurrentJointPosition().get();
            // convert array to list
            for (int i=0; i<jnts.length; i++){
            	jntsmove.add(jnts[i]*180.0/Math.PI);
            }
            sendArray(jntsmove);
            try {
            	Thread.sleep(1000/REFRESH_RATE);
            } catch (InterruptedException e){
            	System.out.println("Pause Interrupted");
            	while (!robot.isReadyToMove()){
            		// just wait...
            	}
            	return;
            }
    	}
    }

    public void run() {
        int port = DRIVER_PORT;
        String ip = "172.31.1.147";
        Boolean keepAlive = true;

        List<Double> values = new ArrayList<Double>();
        List<Double> values2 = new ArrayList<Double>();


        // Default speed in mm/s
        double speed_mms = 300;
        double speed_degs = 200;
        double accel_mmss = 1500;
        double accel_degss = 300;

        // default blend radius, in mm
        double rounding_mm = 1;

        // create a new tool pose in the tool flange
        TOOL = new Tool("RoboDK Driver Tool");
        TOOL.attachTo(robot.getFlange(), Transformation.ofDeg(0,0,0, 0,0,0));

        /*
        // retrieve the robot IP
        try {
            ip = InetAddress.getLocalHost().getHostName();
            System.out.println("Robot IP: " + ip);
        }
        catch (UnknownHostException e) {
            System.out.println("Unable to retrieve robot IP");
            e.printStackTrace();
        }
        */

        //create the socket server object
        while (true){
            try {
                server = new ServerSocket(port);
                break;
            }
            catch (SocketException e) {
                System.out.println("Unable to open port" + port);
                port += 1;
            }
            catch (IOException e) {
                e.printStackTrace();
            }

        }

        // Show status information
        System.out.println("RoboDK Driver running");
        System.out.println("Robot IP: " + ip);
        System.out.println("Robot Port: " + port);
        System.out.println();

        // Run the RoboDK driver server
        while (keepAlive){
            try {
                Boolean keep_connected = true;
                // wait for a new connection:
                System.out.println("Waiting for RoboDK to connect to this robot: " + ip + ":" + port);
                client = server.accept();
                System.out.println("Client connected: " + client.getInetAddress().getHostAddress() + " (" + client.getInetAddress().getHostName() + ")");

                //read from socket to ObjectInputStream object
                dis = new DataInputStream(client.getInputStream());
                dos = new DataOutputStream(client.getOutputStream());
                //dis.readLine()

                String response = readLine();
                if (!response.contains("RoboDK")){
                    System.out.println("Unknown client connected. Resetting connection.");
                    continue;
                }
                System.out.println("Hello from KUKA IIWA robot");
                sendLine("Hello from KUKA IIWA robot"); // send to client PC


                while (keep_connected){
                    // wait for new command (blocking)
                    int cmd = readInt();
                    System.out.println("cmd: " + cmd);
                    switch (cmd){
                        case MSG_CJNT:{
                            double jnts[] = robot.getCurrentJointPosition().get();
                            // convert array to list
                            values.clear();
                            for (int i=0; i<jnts.length; i++){
                            	values.add(jnts[i]*180.0/Math.PI);
                            }
                            sendArray(values);
                            break;
                        }
                        case MSG_SETTOOL:{
                        	// read xyzabc
                            values = readArray(6);
                            TOOL.attachTo(robot.getFlange(), Transformation.ofDeg(values.get(0),values.get(1),values.get(2),values.get(3),values.get(4),values.get(5)));
                            sendInt(MSG_ACKNOWLEDGE);
                            break;
                        }
                        case MSG_SPEED:{
                            values = readArray(4);
                            if (values.get(0) > 0){
                                speed_mms = values.get(0);
                            }
                            if (values.get(1) > 0){
                                speed_degs = values.get(1);
                            }
                            if (values.get(2) > 0){
                                accel_mmss = values.get(2);
                            }
                            if (values.get(3) > 0){
                                accel_degss = values.get(3);
                            }
                            sendInt(MSG_ACKNOWLEDGE);
                            break;
                        }
                        case MSG_ROUNDING:{
                            values = readArray(1);
                            rounding_mm = values.get(0);
                            sendInt(MSG_ACKNOWLEDGE);
                            break;
                        }
                        case MSG_MOVEJ:{
                            values = readArray(7);
                            PTP moveptp = ptp(values.get(0)*Math.PI/180.0,
                                    values.get(1)*Math.PI/180.0,
                                    values.get(2)*Math.PI/180.0,
                                    values.get(3)*Math.PI/180.0,
                                    values.get(4)*Math.PI/180.0,
                                    values.get(5)*Math.PI/180.0,
                                    values.get(6)*Math.PI/180.0);
                             moveptp.setJointVelocityRel(0.2);  // set joint speed as %
                            TOOL.move(moveptp);
                            // FIXME: TOOL.moveAsync(moveptp); waitMove();
                            sendInt(MSG_ACKNOWLEDGE);
                            break;
                        }
                        case MSG_MOVEL:{
                            //values = readArray(7);
                            // by joints
                            //LIN movelin = lin(values.get(0)*Math.PI/180.0, values.get(1)*Math.PI/180.0, values.get(2)*Math.PI/180.0, values.get(3)*Math.PI/180.0, values.get(4)*Math.PI/180.0, values.get(5)*Math.PI/180.0, values.get(6)*Math.PI/180.0);
                            // by XYZABC (TCP point with respect to robot base)
                        	values = readArray(13);
                        	Frame frame = new Frame(values.get(7), values.get(8), values.get(9), values.get(10)*Math.PI/180.0, values.get(11)*Math.PI/180.0, values.get(12)*Math.PI/180.0);
                            LIN movelin = lin(frame);
                            movelin.setCartVelocity(speed_mms).setBlendingCart(rounding_mm);
                            double speed_deg_rel = Math.min(1, speed_degs/180);  // max 180 deg/sec
                            movelin.setJointVelocityRel(speed_deg_rel);
                            movelin.setCartAcceleration(accel_mmss);
                            if (forceConditionSingleUse != null) {
                                movelin.breakWhen(forceConditionSingleUse);
                                System.out.println("Using singel use force condition: " + forceConditionSingleUse);
                                forceConditionSingleUse = null;
                            }
                            TOOL.move(movelin);
                            // FIXME: TOOL.moveAsync(movelin); waitMove();
                            sendInt(MSG_ACKNOWLEDGE);
                            break;
                        }
                        case MSG_MOVEL_SEARCH:{
                            System.out.println("WARNING! Move Linear Search move not implemented!");
                            //values = readArray(7);
                            // by joints
                            //LIN movelin = lin(values.get(0)*Math.PI/180.0, values.get(1)*Math.PI/180.0, values.get(2)*Math.PI/180.0, values.get(3)*Math.PI/180.0, values.get(4)*Math.PI/180.0, values.get(5)*Math.PI/180.0, values.get(6)*Math.PI/180.0);
                            // by XYZABC (TCP point with respect to robot base)
                        	values = readArray(13);
                        	Frame frame = new Frame(values.get(7), values.get(8), values.get(9), values.get(10)*Math.PI/180.0, values.get(11)*Math.PI/180.0, values.get(12)*Math.PI/180.0);
                            LIN movelin = lin(frame);
                            movelin.setCartVelocity(speed_mms).setBlendingCart(rounding_mm);
                            movelin.setCartAcceleration(accel_mmss);
                            //TOOL.move();
                            TOOL.moveAsync(movelin); waitMove();
                            sendInt(MSG_ACKNOWLEDGE);
                            // send contact joints
                            sendArray(values);
                            break;
                        }
                        case MSG_MOVEC:{
                            values = readArray(6);
                            values2 = readArray(6);
                            Frame framecirc1 = new Frame(values.get(0), values.get(1), values.get(2), values.get(3)*Math.PI/180.0, values.get(4)*Math.PI/180.0, values.get(5)*Math.PI/180.0);
                            Frame framecirc2 = new Frame(values2.get(0), values2.get(1), values2.get(2), values2.get(3)*Math.PI/180.0, values2.get(4)*Math.PI/180.0, values2.get(5)*Math.PI/180.0);
                            CIRC movec = circ(framecirc1, framecirc2).setCartVelocity(speed_mms).setBlendingCart(rounding_mm);
                            //TOOL.move(movec);
                            TOOL.moveAsync(movec); waitMove();
                            sendInt(MSG_ACKNOWLEDGE);
                            break;
                        }
                        case MSG_HAND_GUIDING:{
                        	robot.setESMState("2");
                        	robot.move(handGuiding());
                        	robot.setESMState("1");
                        	sendInt(MSG_ACKNOWLEDGE);
                            break;
                        }
                        case MSG_POPUP:{
                            String msg_popup = readLine();
                            System.out.println("Popup message: " + msg_popup);
                            getLogger().info("Popup message: " + msg_popup);
                            sendInt(MSG_ACKNOWLEDGE);
                            break;
                        }
                        case MSG_PAUSE:{
                            values = readArray(1);
                            int pause_ms = (int) Math.round(values.get(0));
                            try{
                                Thread.sleep(pause_ms);
                            } catch (InterruptedException e){
                                System.out.println("Pause Interrupted");
                                break;
                            }
                            sendInt(MSG_ACKNOWLEDGE);
                            break;
                        }
                        case MSG_RUNPROG:{
                            int prg_id = readInt();
                            System.out.println("prg_id: " + prg_id);
                            String prg_name = readLine();
                            prg_name = prg_name.replace("<br>", "");
                            System.out.println("Running program SubProgs." + prg_name);
                            Method method = SubPrograms.class.getMethod(prg_name);
                            method.invoke(subprogs);
                            sendInt(MSG_ACKNOWLEDGE);
                            break;
                        }
                        case MSG_SETDO:{
                            values = readArray(2); // {DO_ID, DO_VALUE}
                            System.out.println("values: " + values.toString());
                            int doId = (int) Math.round(values.get(0));
                            boolean doValue = (values.get(1) != 0.0);
                            switch (doId) {
                                case 0: robotIOs.setOutput1(doValue); break;
                            }
                            sendInt(MSG_ACKNOWLEDGE);
                            break;
                        }
                        /*case MSG_WAITDI:{
                            values = readArray(2); // {DI_ID, DO_VALUE}
                            // System.out.println("MSG_WAITDI Instruction not implemented");
                            int diId = (int) Math.round(values.get(0));
                            boolean diValue = (values.get(1) != 0.0);
                            while (robotIOs.getDI(diId) == diValue){
                                Thread.sleep(1);
                            }
                            sendInt(MSG_ACKNOWLEDGE);
                            break;
                        }
                        */
                        case MSG_WAITDI:{
                            values = readArray(2); // {DI_ID, DO_VALUE}
                            System.out.println("MSG_WAITDI Instruction not implemented");
                            sendInt(MSG_ACKNOWLEDGE);
                            keep_connected = false; // STOP CONNECTION!!!
                            break;
                        }
                        case MSG_GETDI:{
                            values = readArray(1); // {DI_ID, }
                            System.out.println("MSG_GETDI Instruction not implemented");
                            List<Double> di_val = Arrays.asList(0.0, 0.0);
                            di_val.set(0, 0.0d);
                            sendArray(di_val);
                            sendInt(MSG_ACKNOWLEDGE);
                            keep_connected = false; // STOP CONNECTION!!!
                            break;
                        }
                        case MSG_GETAI:{
                            values = readArray(1); // {AI_ID, }
                            System.out.println("MSG_GETAI Instruction not implemented");
                            List<Double> ai_val = Arrays.asList(0.0, 0.0);
                            ai_val.set(0, 0.0d);
                            sendArray(ai_val);
                            sendInt(MSG_ACKNOWLEDGE);
                            keep_connected = false; // STOP CONNECTION!!!
                            break;
                        }
                        case MSG_DISCONNECT:{
                            sendInt(MSG_ACKNOWLEDGE);
                            keep_connected = false; // STOP CONNECTION!!!
                            break;
                        }
                        default:
                            // Unknown instruction: stop current thread
                            System.out.println("Unknown instruction received: " + cmd);
                            dis.close();
                            dos.close();
                            client.close();
                            keepAlive = false;
                            continue;
                    }
                }
            } catch (SocketException e){
                // connection may have been closed abruptly
                if (isDisposed)  // driver got stopped by the user
                    keepAlive = false;
                else
                    e.printStackTrace();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        System.out.println("Shutting down RoboDK Driver");
        try {
            server.close();
        } catch (IOException e){
            e.printStackTrace();
            System.out.println("Unable to close the server");
        }
    }
}
