package com.comino.realsense.boofcv.odometry;

import boofcv.abst.sfm.d3.DepthVisualOdometry;
import boofcv.struct.image.ImageBase;
import boofcv.struct.image.ImageGray;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

public interface RealSenseDepthVisualOdometry<Vis extends ImageBase, Depth extends ImageGray>
               extends DepthVisualOdometry<Vis, Depth> {

	public int getInlierCount();

	public Point3D_F64 getTrackLocation(int index);

	public double getQuality();

	public void reset(Se3_F64 initialState);

	public void setRotation(Se3_F64 rotation);


}
