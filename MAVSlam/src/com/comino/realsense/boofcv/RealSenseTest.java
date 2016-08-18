package com.comino.realsense.boofcv;

import com.comino.librealsense.wrapper.LibRealSenseWrapper;

public class RealSenseTest {

	private StreamRealSenseVisDepth realsense;

	public RealSenseTest() {
		RealSenseInfo info = new RealSenseInfo(320,240, RealSenseInfo.MODE_RGB);
		try {

			System.out.println("Using: "+LibRealSenseWrapper.INSTANCE.JNA_LIBRARY_NAME);
			realsense = new StreamRealSenseVisDepth(0,info);

			} catch(Exception e) {
				System.out.println("REALSENSE:"+e.getMessage());
				return;
			}
	}

	public static void main(String[] args) {
		new RealSenseTest();
	}

}
