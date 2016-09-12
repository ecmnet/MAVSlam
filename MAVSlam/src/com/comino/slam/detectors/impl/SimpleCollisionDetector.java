package com.comino.slam.detectors.impl;

import java.awt.Color;

import org.mavlink.messages.MAV_SEVERITY;

import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.model.segment.LogMessage;
import com.comino.slam.detectors.ISLAMDetector;

import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class SimpleCollisionDetector implements ISLAMDetector {

	private IMAVMSPController control;
    private long tms = 0;


	public SimpleCollisionDetector(IMAVMSPController control) {
		this.control = control;

	}

	@Override
	public void process(AccessPointTracks3D points, GrayU16 depth, Planar<GrayU8> rgb) {
       int total = 0; int count = 0;int x = 0; int y = 0; int d = 0;
		for( int i = 0; i < points.getAllTracks().size(); i++ ) {
			if(points.isInlier(i)) {
				x = (int)points.getAllTracks().get(i).x;
				y = (int)points.getAllTracks().get(i).y;
				d = depth.get(x,y);
				total++;
				if(d<500) count++;
			}
		}
		if((count / total)>0.6f && (System.currentTimeMillis() - tms) > 500) {
		   control.writeLogMessage(new LogMessage("[vis] Collision warning",
					MAV_SEVERITY.MAV_SEVERITY_WARNING));
		   tms = System.currentTimeMillis();
		}
	}
}
