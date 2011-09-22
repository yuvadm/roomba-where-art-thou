package com.hackingroomba.roombacomm;

public class IdoTest1 {

	/**
	 * @param args
	 * @throws Exception 
	 */
	public static void main(String[] args) throws Exception {

		RoombaCommSerial roomba = new RoombaCommSerial();
		    String[] ports = roomba.listPorts();               // if implemented
		    roomba.setProtocol("OI");
		    roomba.connect("/dev/ttyUSB0");
		    roomba.startup();
		    //roomba.control();
//		    roomba.send(new byte[] {(byte) 136, (byte) 9});
//		    Thread.sleep((long)15);
//		    roomba.send(new byte[] {(byte) 141, (byte) 1});
		    //roomba.send(9);
	        //roomba.max();
	        //roomba.send(new int[] {( 136, 0});
			//
		    byte cmd[] = { (byte)roomba.SENSORS, (byte)0};
		    boolean success = roomba.send(cmd);
		    String s = roomba.getSensorsAsString();
		    System.out.println(s);
		    //boolean success = roomba.updateSensors();
		    //roomba.send(new byte[] {(byte)148, (byte)1, (byte)2});
		    //roomba.getSensorData(4);
		    //roomba.sensors(2);
		    roomba.send(new byte[] {(byte) 136, (byte) 255});
		    roomba.disconnect();

	}

}
