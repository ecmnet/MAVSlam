package com.comino.realsense.boofcv.odometry;

import boofcv.abst.sfm.d3.DepthVisualOdometry;
import boofcv.struct.image.ImageBase;
import boofcv.struct.image.ImageGray;

public interface RealSenseDepthVisualOdometry<Vis extends ImageBase, Depth extends ImageGray>
               extends DepthVisualOdometry<Vis, Depth> {

	/**
	 * Get the quality of current estimation (0..100)
	 */
	public int getQuality();

}
