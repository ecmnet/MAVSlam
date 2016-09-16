package com.comino.slam.detectors;

import com.comino.realsense.boofcv.odometry.RealSenseDepthVisualOdometry;

import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public interface ISLAMDetector {

	public void process(RealSenseDepthVisualOdometry<GrayU8,GrayU16> odometry, GrayU16 depth, Planar<GrayU8> rgb, int quality);

}
