package com.comino.slam.estimators;

import com.comino.server.mjpeg.IVisualStreamHandler;
import com.comino.slam.boofcv.MAVDepthVisualOdometry;
import com.comino.slam.detectors.ISLAMDetector;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;

public interface IPositionEstimator {

	void registerDetector(ISLAMDetector detector);

	void registerStreams(IVisualStreamHandler stream);

	void start();

	void stop();

	boolean isRunning();

	void reset();

	void enableDetectors( boolean enable);

	MAVDepthVisualOdometry<GrayU8,GrayU16> getOdometry();

}