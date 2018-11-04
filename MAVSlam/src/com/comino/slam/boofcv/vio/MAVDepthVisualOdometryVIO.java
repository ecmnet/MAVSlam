package com.comino.slam.boofcv.vio;

import boofcv.abst.sfm.d3.DepthVisualOdometry;
import boofcv.struct.image.ImageBase;
import boofcv.struct.image.ImageGray;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

public interface MAVDepthVisualOdometryVIO<Vis extends ImageBase, Depth extends ImageGray>
               extends DepthVisualOdometry<Vis, Depth> {

	public int getInlierCount();

	public Point3D_F64 getTrackLocation(int index);

	public double getQuality();

	public void reset(Se3_F64 initialState);

	public boolean process(Vis visual, Depth depth, Se3_F64 currentAttitude);

	public Point3D_F64 getPoint3DFromPixel(int pixelx, int pixely);

}
