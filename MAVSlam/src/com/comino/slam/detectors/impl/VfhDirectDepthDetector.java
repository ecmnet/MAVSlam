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


import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.lquac.msg_msp_command;

import com.comino.main.MSPConfig;
import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.execution.autopilot.Autopilot2D;
import com.comino.msp.execution.control.listener.IMAVLinkListener;
import com.comino.msp.model.DataModel;
import com.comino.msp.slam.map2D.ILocalMap;
import com.comino.msp.utils.MSP3DUtils;
import com.comino.server.mjpeg.IVisualStreamHandler;
import com.comino.slam.boofcv.odometry.MAVDepthVisualOdometry;
import com.comino.slam.detectors.ISLAMDetector;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.transform.se.SePointOps_F64;

public class VfhDirectDepthDetector implements ISLAMDetector {

	private float     	max_distance     = 3.0f;
	private float     	min_altitude     = 0.35f;

	private DataModel   model        	= null;
	private ILocalMap 	map 				= null;

	private Point3D_F64	point			= null;
	private Point3D_F64  point_ned       = new Point3D_F64();

	private Se3_F64 		current 		 	= new Se3_F64();

	private IMAVMSPController control = null;

	public VfhDirectDepthDetector(IMAVMSPController control, MSPConfig config, IVisualStreamHandler streamer) {

		this.model   = control.getCurrentModel();
		this.control  = control;
		this.map = Autopilot2D.getInstance().getMap2D();

		this.max_distance = config.getFloatProperty("feature_max_distance", "3.00f");
		System.out.println("[col] Max planning distance set to "+max_distance);
		this.min_altitude = config.getFloatProperty("faeture_min_altitude", "0.3f");
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

	}

	@Override
	public void process(MAVDepthVisualOdometry<GrayU8,GrayU16> odometry, GrayU16 depth, GrayU8 gray) {

		getModelToState(model,current);

	//	int y = gray.getHeight()/2;

				int y0 = 2*gray.getHeight()/3; int y1 = y0 + 10;
				for(int y = y0;y<y1;y++)

		for(int x = 0;x < gray.getWidth();x++) {
			point = odometry.getPoint3DFromPixel(x, y);

			if(point==null || point.z > 6.0f)
				continue;

			SePointOps_F64.transform(current,point,point_ned);
			MSP3DUtils.toNED(point_ned);
			map.update(model.state.l_x, model.state.l_y,point_ned);
		}
	}


	public void reset(float x, float y, float z) {

	}

	private Se3_F64 getModelToState(DataModel m, Se3_F64 state) {
		ConvertRotation3D_F64.eulerToMatrix(EulerType.ZXY,
				m.attitude.r,
				m.attitude.p,
				m.attitude.y,
				state.getRotation());

		state.getTranslation().y = m.state.l_z;
		state.getTranslation().x = m.state.l_y;
		state.getTranslation().z = m.state.l_x;

		return state;
	}




}
