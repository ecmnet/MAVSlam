package com.comino.slam.detectors.impl;


import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.model.DataModel;
import com.comino.msp.model.segment.LogMessage;
import com.comino.slam.detectors.ISLAMDetector;
import com.comino.slam.detectors.space.AttributedPoint3D_F32;
import com.comino.slam.detectors.space.NavigationSpace;

import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.struct.image.Color3_I32;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.struct.point.Point3D_F32;

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
	public void process(AccessPointTracks3D points, GrayU16 depth, Planar<GrayU8> rgb, int quality) {
		int x = 0; int y = 0; int z = 0;

		if(quality < 10)
			return;

		obstacles.clear();
		obstacles.setOrigin(model.state.l_x, model.state.l_y, model.state.l_z);

		for( int i = 0; i < points.getAllTracks().size(); i++ ) {
			if(points.isInlier(i)) {
				x = (int)points.getAllTracks().get(i).x;
				y = (int)points.getAllTracks().get(i).y;
				z = depth.get(x,y);

				// TODO: Convert Features to World coordinates and normalize

				Color3_I32 c = new Color3_I32();
				c.band0 = rgb.bands[0].get(x, y);
				c.band1 = rgb.bands[1].get(x, y);
				c.band2 = rgb.bands[2].get(x, y);

				obstacles.addFeature(new AttributedPoint3D_F32(x,y,z,c));

			}
		}

	   Point3D_F32 obstacle_pos = obstacles.getMaxFeaturesPositionWorld();
	   if(obstacle_pos!=null)
		   control.writeLogMessage(
			  new LogMessage("[slam] Possible collision at ("+obstacle_pos.x+","+obstacle_pos.y+","+obstacle_pos.z+")",
					MAV_SEVERITY.MAV_SEVERITY_CRITICAL));

	}

}
