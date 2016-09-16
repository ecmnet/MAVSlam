package com.comino.slam.detectors.impl;


import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.model.DataModel;
import com.comino.msp.model.segment.LogMessage;
import com.comino.realsense.boofcv.odometry.RealSenseDepthVisualOdometry;
import com.comino.slam.detectors.ISLAMDetector;
import com.comino.slam.detectors.space.Feature;
import com.comino.slam.detectors.space.NavigationSpace;

import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.struct.image.Color3_I32;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.point.Point3D_F32;
import georegression.struct.point.Point3D_F64;

public class SegmentedCollisionDetector implements ISLAMDetector {



	private IMAVMSPController control;
	private DataModel         model;
	private NavigationSpace   obstacles;

	public SegmentedCollisionDetector(IMAVMSPController control) {
		this.control  = control;
		this.model    = control.getCurrentModel();
		this.obstacles = new NavigationSpace();
	}

	@Override
	public void process(RealSenseDepthVisualOdometry<GrayU8,GrayU16> odometry, GrayU16 depth, Planar<GrayU8> rgb, int quality) {
		int x = 0; int y = 0; int z = 0;

		if(quality < 10)
			return;

		obstacles.clear();
		obstacles.setOrigin(model.state.l_x, model.state.l_y, model.state.l_z);

		for( int i = 0; i < odometry.getInlierCount(); i++ ) {

				Point3D_F64 point = odometry.getTrackLocation(i);
				obstacles.addFeature(new Feature(point.x,point.y,point.z));


		}

	   Point3D_F32 obstacle_pos = obstacles.getMaxFeaturesPositionWorld();
	   if(obstacle_pos!=null)
		   control.writeLogMessage(
			  new LogMessage("[slam] Possible collision at ("+obstacle_pos.x+","+obstacle_pos.y+","+obstacle_pos.z+")",
					MAV_SEVERITY.MAV_SEVERITY_CRITICAL));

	}

}
