package com.comino.slam.detectors;

import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public interface ISLAMDetector {

	public void process(AccessPointTracks3D points, GrayU16 depth, Planar<GrayU8> rgb, int quality);

}
