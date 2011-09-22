package com.hackingroomba.roombacomm;

public class IdoTest1 {

	/**
	 * @param args
	 */
	public static void main(String[] args) {

		RoombaCommSerial roomba = new RoombaCommSerial();
		    String[] ports = roomba.listPorts();               // if implemented
		    roomba.connect("/dev/ttyUSB0");
		    roomba.startup();
			roomba.control();
		    roomba.updateSensors();
		    {
		       roomba.sensors();
		       
		       roomba.playNote( 53, 12 );
		       roomba.goForward( 400 );
		       roomba.spinRight( 45 );
		       if( roomba.bump() ) roomba.goBackward( 100 );
		    }    
		    roomba.disconnect();

	}

}
