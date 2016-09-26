/****************************************************************************
 *
 *   Copyright (c) 2016 Eike Mansfeld ecm@gmx.de. All rights reserved.
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

package com.comino.slam.estimators;


import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.MSP_COMPONENT_CTRL;
import org.mavlink.messages.lquac.msg_msp_command;
import org.mavlink.messages.lquac.msg_msp_vision;
import org.mavlink.messages.lquac.msg_vision_position_estimate;

import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.main.MSPConfig;
import com.comino.msp.main.control.listener.IMAVLinkListener;
import com.comino.msp.model.DataModel;
import com.comino.msp.model.segment.LogMessage;
import com.comino.msp.utils.MSPMathUtils;
import com.comino.realsense.boofcv.RealSenseInfo;
import com.comino.realsense.boofcv.StreamRealSenseVisDepth;
import com.comino.realsense.boofcv.StreamRealSenseVisDepth.Listener;
import com.comino.realsense.boofcv.odometry.FactoryRealSenseOdometry;
import com.comino.realsense.boofcv.odometry.RealSenseDepthVisualOdometry;
import com.comino.server.mjpeg.MJPEGHandler;
import com.comino.slam.detectors.ISLAMDetector;
import com.comino.slam.model.RotationModel;

import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
import boofcv.abst.feature.tracker.PointTrackerTwoPass;
import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.alg.distort.DoNothingPixelTransform_F32;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.alg.tracker.klt.PkltConfig;
import boofcv.factory.feature.tracker.FactoryPointTrackerTwoPass;
import boofcv.struct.image.GrayS16;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.geometry.GeometryMath_F32;
import georegression.struct.EulerType;
import georegression.struct.point.Vector3D_F32;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

public class RealSensePositionEstimator {

	private static final int    INIT_TIME_MS    	= 100;

	private static final float  MAX_SPEED   		= 2;

	private static final float  MAX_ROT_SPEED   	= 2f;
	private static final float  MAX_ROTATION 		= MSPMathUtils.toRad(5f);

	private static final int    MIN_QUALITY 		= 15;
	private static final int    MAXTRACKS   		= 130;
	private static final int    RANSAC_ITERATIONS   = 250;

	private StreamRealSenseVisDepth realsense;
	private RealSenseDepthVisualOdometry<GrayU8,GrayU16> visualOdometry;

	private long oldTimeDepth=0;

	private Vector3D_F64 pos_raw;
	private Vector3D_F64 pos_raw_old = new Vector3D_F64();

	private Vector3D_F32 pos         = new Vector3D_F32();
	private Vector3D_F32 pos_ned     = new Vector3D_F32();

	private Vector3D_F32 speed       = new Vector3D_F32();
	private Vector3D_F32 speed_old   = new Vector3D_F32();
	private Vector3D_F32 speed_ned   = new Vector3D_F32();

	private Vector3D_F32 cam_offset  = new Vector3D_F32();

	private RotationModel rotation   = new RotationModel();

	private long fps_tms   =0;
	private long init_tms  =0;

	private DataModel model;

	private boolean debug = false;

	private boolean isRunning = false;
	private IMAVMSPController control;

	private double[] init_rot_ned		= new double[3];
	private double[] vis_att_ned  		= new double[3];

	private float init_offset_rad = 0;

	private int error_count = 0;
	private int init_count = 0;

	private boolean do_odometry = true;

	private long detector_tms = 0;
	private int  detector_cycle_ms = 250;

	private List<ISLAMDetector> detectors = null;

	public RealSensePositionEstimator(RealSenseInfo info, IMAVMSPController control, MSPConfig config, MJPEGHandler streamer ) {

		this.control = control;
		this.detectors = new ArrayList<ISLAMDetector>();

		this.debug = config.getBoolProperty("vision_debug", "false");
		System.out.println("Vision debugging: "+debug);

		this.detector_cycle_ms = config.getIntProperty("vision_detector_cycle", "250");
		if(this.detector_cycle_ms>0)
			System.out.printf("Vision detectors enablied with %2 [ms] cycle \n",detector_cycle_ms);

		this.init_offset_rad = MSPMathUtils.toRad(config.getFloatProperty("vision_rot_offset", "0.0"));
		System.out.printf("Vision rotation offset: %2.3f [rad]\n",init_offset_rad);

		this.cam_offset.z = -config.getFloatProperty("vision_x_offset", "0.0");
		this.cam_offset.x = -config.getFloatProperty("vision_y_offset", "0.0");
		this.cam_offset.y = -config.getFloatProperty("vision_z_offset", "0.0");

		System.out.printf("Vision position offset: %.3f,%.3f,%.3f [m]\n",this.cam_offset.x,this.cam_offset.y,this.cam_offset.z);

		this.model = control.getCurrentModel();

		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				msg_msp_command cmd = (msg_msp_command)o;
				switch(cmd.command) {
				case MSP_CMD.MSP_CMD_VISION:
					if((int)(cmd.param1)==MSP_COMPONENT_CTRL.ENABLE) {
						do_odometry = true; init("Init"); break;
					}
					if((int)(cmd.param1)==MSP_COMPONENT_CTRL.DISABLE) {
						do_odometry = false; break; };
						break;
				}
			}
		});

		try {
			realsense = new StreamRealSenseVisDepth(0,info);
		} catch(Exception e) {

		}

		PkltConfig configKlt = new PkltConfig();
		configKlt.pyramidScaling = new int[]{1, 2, 4, 8};
		configKlt.templateRadius = 3;

		PointTrackerTwoPass<GrayU8> tracker =
				FactoryPointTrackerTwoPass.klt(configKlt, new ConfigGeneralDetector(MAXTRACKS, 3, 1),
						GrayU8.class, GrayS16.class);

		DepthSparse3D<GrayU16> sparseDepth = new DepthSparse3D.I<GrayU16>(1e-3);

		visualOdometry = FactoryRealSenseOdometry.depthDepthPnP(1.2, 120, 2, RANSAC_ITERATIONS, 50, true,
				sparseDepth, tracker, GrayU8.class, GrayU16.class);

		visualOdometry.setCalibration(realsense.getIntrinsics(),new DoNothingPixelTransform_F32());

		if(debug) {
			streamer.registerOverlayListener(ctx -> {
				overlayFeatures(ctx);
			});
		}

		init_count = 0;
		init_tms = System.currentTimeMillis();

		realsense.registerListener(new Listener() {

			float fps; float dt; int mf=0; int fpm;
			Se3_F64 leftToWorld; float ang_speed; float odo_speed;

			@Override
			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {


				dt = (timeDepth - oldTimeDepth)/1000f;
				oldTimeDepth = timeDepth;

				fpm += (int)(1f/dt+0.5f);
				if((System.currentTimeMillis() - fps_tms) > 500) {
					fps_tms = System.currentTimeMillis();
					if(mf>0)
						fps = fpm/mf;
					mf=0; fpm=0;
				}
				mf++;

				if(streamer!=null)
					streamer.addImage(rgb.bands[0]);

				if(!do_odometry) {
					msg_msp_vision msg = new msg_msp_vision(1,2);
					msg.x =  Float.NaN;
					msg.y =  Float.NaN;
					msg.z =  Float.NaN;
					msg.vx = Float.NaN;
					msg.vy = Float.NaN;
					msg.vz = Float.NaN;
					msg.h = MSPMathUtils.fromRad((float)init_rot_ned[RotationModel.YAW]);
					msg.quality = rotation.quality;
					msg.fps = fps;
					msg.errors = error_count;
					msg.flags = msg.flags & 1;
					msg.tms = System.nanoTime() / 1000;
					control.sendMAVLinkMessage(msg);
					return;
				}

				// Check PX4 rotation and reset odometry if rotating too fast
				ang_speed = (float)Math.sqrt(model.attitude.pr * model.attitude.pr +
						                     model.attitude.rr * model.attitude.rr +
						                     model.attitude.yr * model.attitude.yr);

				if(ang_speed > MAX_ROT_SPEED) {
					if(debug)
						System.out.println("[vis] Rotation "+ang_speed+" > MAX");
					init("IMU.Rot.speed");
					return;
				}


				if( !visualOdometry.process(rgb.getBand(0),depth) ) {
					if(debug)
						System.out.println("[vis] Odometry failure");
					init("Odometry");
					return;
				}

				if((System.currentTimeMillis()-init_tms) < INIT_TIME_MS) {

					init_rot_ned[RotationModel.PITCH] = (init_rot_ned[RotationModel.PITCH] * init_count + model.attitude.p );
					init_rot_ned[RotationModel.ROLL]  = (init_rot_ned[RotationModel.ROLL]  * init_count + model.attitude.r );
					init_rot_ned[RotationModel.YAW]   = (init_rot_ned[RotationModel.YAW]   * init_count + model.attitude.y );

					init_count++;

					init_rot_ned[RotationModel.PITCH] = -init_rot_ned[RotationModel.PITCH] / init_count;
					init_rot_ned[RotationModel.ROLL]  = -init_rot_ned[RotationModel.ROLL]  / init_count;
					init_rot_ned[RotationModel.YAW]   =  init_rot_ned[RotationModel.YAW]   / init_count +init_offset_rad;

					rotation.setVIS(init_rot_ned);

					pos.set(0,0,0);
					speed_old.set(0,0,0);
					return;
				}

				leftToWorld = visualOdometry.getCameraToWorld();
				pos_raw = leftToWorld.getT();
				rotation.setPOS(leftToWorld.getRotation());

				rotation.quality = visualOdometry.getInlierCount() *100 / MAXTRACKS ;

				if(pos_raw_old!=null) {

					if(rotation.quality > MIN_QUALITY ) {

						speed.x =  (float)(pos_raw.x - pos_raw_old.x)/dt;
						speed.y =  (float)(pos_raw.y - pos_raw_old.y)/dt;
						speed.z =  (float)(pos_raw.z - pos_raw_old.z)/dt;

					} else {
						if(debug)
							System.out.println("[vis] Quality "+rotation.quality+" < Min");

						error_count++;
						return;
					}

					odo_speed = (float) Math.sqrt(speed.x * speed.x +
							speed.y * speed.y +
							speed.z * speed.z);

					if(odo_speed < MAX_SPEED) {

						pos.x += speed.x * dt;
						pos.y += speed.y * dt;
						pos.z += speed.z * dt;


					} else {

						if(debug)
							System.out.println("[vis] Speed  "+odo_speed+" > MAX");

						pos.x += speed_old.x * dt;
						pos.y += speed_old.y * dt;
						pos.z += speed_old.z * dt;

						error_count--;
					}

					speed_old.set(speed);
				}

				pos_raw_old.x = pos_raw.x;
				pos_raw_old.y = pos_raw.y;
				pos_raw_old.z = pos_raw.z;

				GeometryMath_F32.mult(rotation.R_VIS, pos, pos_ned);
				ConvertRotation3D_F64.matrixToEuler(rotation.R_POS,EulerType.ZXY,vis_att_ned);

				if(Math.abs(vis_att_ned[RotationModel.YAW]  -model.attitude.y)>MAX_ROTATION) {
					if(debug)
						System.out.println("[vis] IMU vs. visual rotation > MAX");
					init("Vis.Rot");
					return;
				}


				if(control!=null) {

					msg_vision_position_estimate sms = new msg_vision_position_estimate(1,1);
					sms.usec = System.nanoTime() / 1000;
					sms.x = (float) pos_ned.z;
					sms.y = (float) pos_ned.x;
					sms.z = (float) pos_ned.y;
					control.sendMAVLinkMessage(sms);

					GeometryMath_F32.mult(rotation.R_VIS, speed, speed_ned);

					msg_msp_vision msg = new msg_msp_vision(1,2);
					msg.x =  (float) pos_ned.z;
					msg.y =  (float) pos_ned.x;
					msg.z =  (float) pos_ned.y;
					msg.vx = (float) speed_ned.z;
					msg.vy = (float) speed_ned.x;
					msg.vz = (float) speed_ned.y;
					msg.h = MSPMathUtils.fromRad((float)vis_att_ned[RotationModel.YAW]);
					msg.quality = rotation.quality;
					msg.fps = fps;
					msg.errors = error_count;
					msg.flags = msg.flags | 1;
					msg.tms = System.nanoTime() / 1000;
					control.sendMAVLinkMessage(msg);
				}

				if(detectors.size()>0 && detector_cycle_ms>0) {


					if((System.currentTimeMillis() - detector_tms) > detector_cycle_ms) {
						detector_tms = System.currentTimeMillis();
						for(ISLAMDetector d : detectors)
							d.process(visualOdometry, depth, rgb, rotation);

					}
				}
			}
		});
	}

	private void overlayFeatures(Graphics ctx) {
		AccessPointTracks3D points = (AccessPointTracks3D)visualOdometry;
		for( int i = 0; i < points.getAllTracks().size(); i++ ) {
			if(points.isInlier(i))
				ctx.drawRect((int)points.getAllTracks().get(i).x,(int)points.getAllTracks().get(i).y, 1, 1);
		}
	}

	public RealSensePositionEstimator() {
		this(new RealSenseInfo(320,240, RealSenseInfo.MODE_RGB), null, MSPConfig.getInstance("msp.properties"),null);
	}

	public void registerDetector(ISLAMDetector detector) {
		if(detector_cycle_ms>0) {
			System.out.println("[vis] Vision detector registered: "+detector.getClass().getSimpleName());
			detectors.add(detector);
		}
	}

	public void start() {
		isRunning = true; init_tms=0;
		init("");
		if(realsense!=null)
			realsense.start();
	}

	public void stop() {
		if(isRunning) {
			realsense.stop();

			msg_msp_vision msg = new msg_msp_vision(1,2);
			msg.x = Float.NaN;
			msg.y = Float.NaN;
			msg.z = Float.NaN;
			msg.h = MSPMathUtils.fromRad((float)vis_att_ned[RotationModel.YAW]);
			msg.quality = 0;
			msg.fps = 0;
			msg.flags = 0;
			msg.tms = System.nanoTime() / 1000;
			control.sendMAVLinkMessage(msg);

		}
		isRunning=false;
	}

	public boolean isRunning() {
		return isRunning;
	}

	private void init(String msg) {
		if((System.currentTimeMillis()-init_tms)>INIT_TIME_MS*3) {
			control.writeLogMessage(new LogMessage("[vis] reset odometry: "+msg,
					MAV_SEVERITY.MAV_SEVERITY_WARNING));
			visualOdometry.reset();
			init_count = 0;
			error_count=0;
			init_rot_ned[RotationModel.PITCH]=0; init_rot_ned[RotationModel.ROLL]=0; init_rot_ned[RotationModel.YAW]=0;
			init_tms = System.currentTimeMillis();
		}
	}

	public static void main(String[] args) {
		new RealSensePositionEstimator();
	}

}
