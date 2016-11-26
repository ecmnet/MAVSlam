package com.comino.server.mjpeg;

import com.comino.msp.model.DataModel;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;

public interface IVisualStreamHandler {

	public final static int HTTPVIDEO = 0;
	public final static int FILE = 1;

	public void addToStream(GrayU8 grayImage, GrayU16 depth, DataModel model, long tms_us);
	public void registerOverlayListener(IMJPEGOverlayListener listener);

}