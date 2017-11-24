/****************************************************************************
 *
 *   Copyright (c) 2017 Eike Mansfeld ecm@gmx.de. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

package com.comino.slam.detectors.impl;


import java.util.ArrayList;
import java.util.List;

import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.lquac.msg_msp_command;

import com.comino.main.MSPConfig;
import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.execution.autopilot.Autopilot2D;
import com.comino.msp.execution.control.listener.IMAVLinkListener;
import com.comino.msp.model.DataModel;
import com.comino.msp.slam.map.ILocalMap;
import com.comino.server.mjpeg.IVisualStreamHandler;
import com.comino.slam.boofcv.odometry.MAVDepthVisualOdometry;
import com.comino.slam.detectors.ISLAMDetector;

import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import georegression.struct.point.Point2D_F64;
import georegression.struct.point.Point3D_F64;
import georegression.struct.point.Vector4D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

public class VfhFeatureDetector implements ISLAMDetector {

	private float     	max_distance     = 3.0f;
	private float     	min_altitude     = 0.35f;

	private DataModel     	model        = null;
	private Point3D_F64   	p_ned        = new Point3D_F64();

	private Se3_F64 			current 		 = new Se3_F64();

	private float debug1 = 0;
	private float debug2 = 0;
	private float debug3 = 0;

	private ILocalMap map = null;

	private Vector4D_F64   current_pos   = new Vector4D_F64();


	private List<Point2D_F64> nearestPoints =  new ArrayList<Point2D_F64>();

	public VfhFeatureDetector(IMAVMSPController control, MSPConfig config, IVisualStreamHandler streamer) {

		this.model   = control.getCurrentModel();

		this.map = Autopilot2D.getInstance().getMap2D();

		this.max_distance = config.getFloatProperty("max_distance", "3.00f");
		System.out.println("[col] Max planning distance set to "+max_distance);
		this.min_altitude = config.getFloatProperty("min_altitude", "0.3f");
		System.out.println("[col] Min.altitude set to "+min_altitude);

		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				msg_msp_command cmd = (msg_msp_command)o;
				switch(cmd.command) {
				case MSP_CMD.MSP_TRANSFER_MICROSLAM:
					model.grid.invalidateTransfer();
					break;
				}
			}
		});

		streamer.registerOverlayListener(ctx -> {
			if(nearestPoints.size()>0) {
				for(Point2D_F64 n : nearestPoints) {
					ctx.drawRect((int)n.x-5, (int)n.y-5, 10,10);
				}
			}
			ctx.drawString(String.format("%.2f",debug1), 20, 35);
			ctx.drawString(String.format("%.2f",debug2), 20, 50);
			ctx.drawString(String.format("%.2f",debug3), 20, 65);
		});

	}

	@Override
	public void process(MAVDepthVisualOdometry<GrayU8,GrayU16> odometry, GrayU16 depth, GrayU8 gray) {
		Point2D_F64 xy; Point3D_F64 p;

		AccessPointTracks3D points = (AccessPointTracks3D)odometry;

		nearestPoints.clear();

		current_pos.set(model.state.l_x, model.state.l_y, model.state.l_z, model.state.h);

		current = odometry.getCameraToWorld();
//		getAttitudeToState(model,current);

//		int i = 0; {
		for( int i = 0; i < points.getAllTracks().size(); i++ ) {
			if(points.isInlier(i)) {

				// xy is the observation
				xy = points.getAllTracks().get(i);
				// p is the obstacle location in body-frame
				p = odometry.getTrackLocation(i);

				SePointOps_F64.transform(current,p,p_ned);

				Point3D_F64 pos = new Point3D_F64(p_ned.z - current.T.z, p_ned.x - current.T.x, -p_ned.y - current.T.y);

				// Search highest point (note: NED coordinate system)

				if(     model.raw.di > min_altitude &&
					    p.z < max_distance && p.y< min_altitude && p.y > -min_altitude) {

					debug1 = (float)pos.x;
					debug2 = (float)pos.y;
					debug3 = (float)model.raw.di;

					map.update(pos, current_pos);

					nearestPoints.add(xy);
				}
			}
		}
	}


	public void reset(float x, float y, float z) {
		nearestPoints.clear();
	}



}
