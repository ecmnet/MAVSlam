package com.comino.realsense.boofcv;

public class RealSenseInfo {

	public static final int MODE_RGB 	  = 0;
	public static final int MODE_INFRARED = 1;

	public int mode   = MODE_RGB;

	public int width  = 480;
	public int height = 360;

	public int framerate = 60;


	public RealSenseInfo() {

	}

	public RealSenseInfo(int width, int height) {
		this.width = width;
		this.height = height;
	}

	public RealSenseInfo(int width, int height, int framerate) {
		this.width = width;
		this.height = height;
		this.framerate = framerate;
	}

	public RealSenseInfo(int width, int height, int framerate, int mode) {
		this.width = width;
		this.height = height;
		this.framerate = framerate;
		this.mode = mode;
	}




}
