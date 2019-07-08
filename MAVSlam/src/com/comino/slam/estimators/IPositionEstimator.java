package com.comino.slam.estimators;

import com.comino.server.mjpeg.IVisualStreamHandler;
import com.comino.slam.detectors.ISLAMDetector;

public interface IPositionEstimator {

	void registerDetector(ISLAMDetector detector);

	void registerStreams(IVisualStreamHandler stream);

	void start();

	void stop();

	boolean isRunning();

	void reset();

	void enableDetectors( boolean enable);

}