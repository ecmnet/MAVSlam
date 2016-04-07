package com.comino.realsense.boofcv;

public class RealSenseInfo {

	public static final int MODE_RGB 	  = 0;
	public static final int MODE_INFRARED = 1;

	public int width  = 320;
	public int height = 240;

	public int framerate = 60;
	public int mode      = 0;


	public RealSenseInfo() {

	}

	public RealSenseInfo(int width, int height) {
		this.width = width;
		this.height = height;
	}

	public RealSenseInfo(int width, int height, int mode) {
		this.width = width;
		this.height = height;
		this.mode = mode;
	}

	public RealSenseInfo(int width, int height, int framerate, int mode) {
		this.width = width;
		this.height = height;
		this.framerate = framerate;
		this.mode = mode;
	}




}
