package com.hackingroomba.roombacomm;

import java.io.InputStream;
import java.io.OutputStream;
import java.util.Enumeration;

import gnu.io.CommPortIdentifier;
import gnu.io.SerialPort;
import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;

public class BetterRoomba implements SerialPortEventListener {
	
	private static final String PORT = "/dev/ttyUSB0";
	private static final int TIMEOUT = 2000;
	
	private static final int RATE = 115200;
	private static final int DATABITS = 8;
	private static final int PARITY = SerialPort.PARITY_NONE;
	private static final int STOPBITS = SerialPort.STOPBITS_1;
	
	private SerialPort serialPort;
	private InputStream input;
	private OutputStream output;
	
	public static void main(String[] args) throws Exception {
		System.out.println("Starting...");
		BetterRoomba br = new BetterRoomba();
		
		br.startup();
		br.send(new byte[] {(byte) 136, (byte) 8});
		Thread.sleep(5000);
		br.send(new byte[] {(byte) 136, (byte) 255});
		br.shutdown();
		
		System.out.println("-END-");
	}
	
	@Override
	public void serialEvent(SerialPortEvent e) {
		// do smthng with e
	}
	
	public void startup() throws Exception {
		serialConnect();
	}
	
	public void shutdown() throws Exception {
		try {
            if (input != null)
            	input.close();
            if (output != null) 
            	output.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
    
        try {
            if (serialPort != null)
            	serialPort.close();
        } catch (Exception e) {
            e.printStackTrace();
        }
	}
	
	private void send(byte[] arr) throws Exception {
		output.write(arr);
	}
	
	private void serialConnect() throws Exception {
		try {
			Enumeration portList = CommPortIdentifier.getPortIdentifiers();
	        while (portList.hasMoreElements()) {
	            CommPortIdentifier portId = (CommPortIdentifier) portList.nextElement();
	            
	            if (portId.getPortType() == CommPortIdentifier.PORT_SERIAL) {
	                System.out.println("Found " + portId.getName());
	                
	                if (portId.getName().equals(PORT)) {
	                	System.out.println("Opening port: "+ portId.getName());
	                    serialPort = (SerialPort) portId.open("roomba serial", TIMEOUT);
	                    
	                    input  = serialPort.getInputStream();
	                    output = serialPort.getOutputStream();
	                    
	                    serialPort.setSerialPortParams(RATE, DATABITS, STOPBITS, PARITY);
	                    serialPort.addEventListener(this);
	                    serialPort.notifyOnDataAvailable(true);
	                    
	                    System.out.println("Port opened successfully");
	                }
	            }
	        }
	    } catch (Exception e) {
	        System.out.println(e.getMessage());
	        throw e;
	    }
	}
}