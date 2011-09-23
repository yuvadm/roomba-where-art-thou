package com.hackingroomba.roombacomm;

import gnu.io.SerialPortEvent;
import gnu.io.SerialPortEventListener;

public class BetterRoomba implements SerialPortEventListener {
	
	private static int RATE = 115200;
	
	public static void main(String[] args) {
		System.out.println("Starting...");
		System.out.println("-END-");
	}
	
	@Override
	public void serialEvent(SerialPortEvent e) {
		// do smthng with e
	}
	
	public void startup() {
		
	}

}
