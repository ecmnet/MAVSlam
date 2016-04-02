package com.comino.realsense.boofcv;

public class RealSenseInfo {

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




}
