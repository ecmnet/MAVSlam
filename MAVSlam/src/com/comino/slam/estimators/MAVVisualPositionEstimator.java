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

package com.comino.slam.estimators;


import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.MSP_COMPONENT_CTRL;
import org.mavlink.messages.lquac.msg_local_position_ned_cov;
import org.mavlink.messages.lquac.msg_msp_command;
import org.mavlink.messages.lquac.msg_msp_vision;

import com.comino.main.MSPConfig;
import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.execution.control.listener.IMAVLinkListener;
import com.comino.msp.model.DataModel;
import com.comino.msp.model.segment.LogMessage;
import com.comino.msp.model.segment.Status;
import com.comino.msp.utils.MSP3DUtils;
import com.comino.msp.utils.MSPMathUtils;
import com.comino.realsense.boofcv.RealSenseInfo;
import com.comino.realsense.boofcv.StreamRealSenseVisDepth;
import com.comino.realsense.boofcv.StreamRealSenseVisDepth.Listener;
import com.comino.server.mjpeg.IVisualStreamHandler;
import com.comino.slam.boofcv.odometry.FactoryMAVOdometry;
import com.comino.slam.boofcv.odometry.MAVDepthVisualOdometry;
import com.comino.slam.boofcv.tracker.FactoryMAVPointTrackerTwoPass;
import com.comino.slam.detectors.ISLAMDetector;

import boofcv.abst.feature.detect.interest.ConfigGeneralDetector;
import boofcv.abst.feature.tracker.PointTrackerTwoPass;
import boofcv.abst.sfm.AccessPointTracks3D;
import boofcv.alg.distort.DoNothingPixelTransform_F32;
import boofcv.alg.sfm.DepthSparse3D;
import boofcv.alg.tracker.klt.PkltConfig;
import boofcv.core.image.ConvertImage;
import boofcv.struct.image.GrayS16;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;
import georegression.geometry.ConvertRotation3D_F64;
import georegression.geometry.GeometryMath_F64;
import georegression.struct.EulerType;
import georegression.struct.point.Vector3D_F64;
import georegression.struct.se.Se3_F64;

public class MAVVisualPositionEstimator implements IPositionEstimator {

	private static final int   	PUBLISH_RATE_MSP	    = 50 - 5;
	private static final int  	PUBLISH_RATE_PX4    	= 20 - 5;

	private static final int    INIT_COUNT           = 1;
	private static final int    MAX_ERRORS    	    = 3;

	private static final int    MAX_SPEED    	    = 20;

	private static final float  INLIER_PIXEL_TOL    = 1.3f;
	private static final int    MAXTRACKS   		   = 150;
	private static final int    KLT_RADIUS          = 3;
	private static final float  KLT_THRESHOLD       = 1f;
	private static final int    RANSAC_ITERATIONS   = 120;
	private static final int    RETIRE_THRESHOLD    = 10;
	private static final int    INLIER_THRESHOLD    = 120;
	private static final int    REFINE_ITERATIONS   = 80;

	// cov[0]	X Covariance
	// cov[9]	Y Covariance
	// cov[17] 	Z Covariance
	// cov[24] 	VX Covariance
	// cov[30] 	VY Covariance
	// cov[35]	VZ Covariance


	private StreamRealSenseVisDepth realsense;
	private MAVDepthVisualOdometry<GrayU8,GrayU16> visualOdometry;

	private GrayU8 gray = null;

	private double oldTimeDepth_us=0;
	private double estTimeDepth_us=0;

	private Vector3D_F64 	pos_raw;
	private Vector3D_F64 pos_raw_old = new Vector3D_F64();

	private Se3_F64 speed_ned        = new Se3_F64();
	private Se3_F64 pos_delta        = new Se3_F64();
	private Se3_F64 pos_ned          = new Se3_F64();

	private Se3_F64 rot_ned          = new Se3_F64();

	private Se3_F64 cam_offset       = new Se3_F64();
	private Se3_F64 cam_offset_ned   = new Se3_F64();


	private Se3_F64 current          = new Se3_F64();

	private double[] visAttitude     = new double[3];
	private long fps_tms             =0;

	private long last_publish_tms    = 0;
	private long last_msp_tms        = 0;
	private long last_msg            = 0;

	private DataModel model;

	private boolean debug = false;
	private boolean heading_init_enabled = false;


	private int quality=0;
	private int min_quality = 0;

	private float fps = 0;

	private boolean isRunning    = false;
	private int     initialized_count  = 0;

	private IMAVMSPController control;

	private int error_count = 0;

	private boolean do_odometry = true;

	private boolean do_xy_position = false;
	private boolean do_z_position  = false;
	private boolean do_xy_speed    = false;
	private boolean do_z_speed     = false;

	private long detector_tms = 0;
	private int  detector_cycle_ms = 250;

	private List<ISLAMDetector> 		detectors = null;
	private List<IVisualStreamHandler>	streams = null;
	private RealSenseInfo info;
	private String last_reason;


	public MAVVisualPositionEstimator(RealSenseInfo info, IMAVMSPController control, MSPConfig config, IVisualStreamHandler stream) {

		this.info    = info;
		this.control = control;
		this.detectors = new ArrayList<ISLAMDetector>();
		this.streams   = new ArrayList<IVisualStreamHandler>();

		System.out.println("Vision position estimator: "+this.getClass().getSimpleName());
		this.debug = config.getBoolProperty("vision_debug", "false");
		this.heading_init_enabled = config.getBoolProperty("vision_heading_init", "true");
		System.out.println("Vision debugging: "+debug);
		System.out.println("Initialize heading when landed: "+heading_init_enabled);
		System.out.println("Vision setup: MaxTracks="+MAXTRACKS+" RanSac="+RANSAC_ITERATIONS+ " KLTRadius="+KLT_RADIUS+ " KLTThreshold="+KLT_THRESHOLD);
		this.min_quality = config.getIntProperty("vision_min_quality", "50");
		System.out.println("Vision minimum quality: "+min_quality);

		this.do_odometry = config.getBoolProperty("vision_enable", "true");
		System.out.println("Vision Odometry enabled: "+do_odometry);
		this.do_xy_speed    = config.getBoolProperty("vision_pub_xy_speed", "false");
		if(this.do_xy_speed) System.out.println("...Vision publishes XY speed ");
		this.do_z_speed    = config.getBoolProperty("vision_pub_z_speed", "false");
		if(this.do_z_speed) System.out.println("...Vision publishes Z speed ");
		this.do_xy_position = config.getBoolProperty("vision_pub_xy_pos", "true");
		if(this.do_xy_position) System.out.println("...Vision publishes XY position ");
		this.do_z_position = config.getBoolProperty("vision_pub_z_pos", "false");
		if(this.do_z_position) System.out.println("...Vision publishes Z position ");


		this.detector_cycle_ms = config.getIntProperty("vision_detector_cycle", "0");
		if(this.detector_cycle_ms > 0)
			System.out.printf("Vision detectors enablied with %d [ms] cycle \n",detector_cycle_ms);

		this.cam_offset.T.z = -config.getFloatProperty("vision_x_offset", "0.0");
		this.cam_offset.T.x = -config.getFloatProperty("vision_y_offset", "0.0");
		this.cam_offset.T.y = -config.getFloatProperty("vision_z_offset", "0.0");

		System.out.printf("Vision position offset [m]: %s\n\n",MSP3DUtils.vector3D_F64ToString(cam_offset.T));
		System.out.println("Resolution: "+info.width+"x"+info.height);

		this.model = control.getCurrentModel();

		gray = new GrayU8(info.width,info.height);

		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				msg_msp_command cmd = (msg_msp_command)o;
				switch(cmd.command) {
				case MSP_CMD.MSP_CMD_VISION:
					switch((int)cmd.param1) {
					case MSP_COMPONENT_CTRL.ENABLE:
						do_odometry = true; init("Init"); break;
					case MSP_COMPONENT_CTRL.DISABLE:
						do_odometry = false; break;
					case MSP_COMPONENT_CTRL.RESET:
						reset(); break;
					}
				}
			}
		});

		// reset vision when armed
		control.getStatusManager().addListener( Status.MSP_ARMED, (o,n) -> {
			if(n.isStatus(Status.MSP_ARMED)) {
				reset();
			}
		});

		// reset vision when GPOS gets valid
		control.getStatusManager().addListener(Status.MSP_GPOS_VALID, (o,n) -> {
			if((n.isStatus(Status.MSP_GPOS_VALID)))
				reset();
		});

		try {
			realsense = new StreamRealSenseVisDepth(0,info);
		} catch(Exception e) {	}

		PkltConfig configKlt = new PkltConfig();
		configKlt.pyramidScaling = new int[]{ 1, 4, 8, 32 };
		configKlt.templateRadius = 3;

		PointTrackerTwoPass<GrayU8> tracker =
				FactoryMAVPointTrackerTwoPass.klt(configKlt, new ConfigGeneralDetector(MAXTRACKS, KLT_RADIUS, KLT_THRESHOLD),
						GrayU8.class, GrayS16.class);

		DepthSparse3D<GrayU16> sparseDepth = new DepthSparse3D.I<GrayU16>(1e-3);


		visualOdometry = FactoryMAVOdometry.depthDepthPnP(INLIER_PIXEL_TOL,
				INLIER_THRESHOLD, RETIRE_THRESHOLD, RANSAC_ITERATIONS, REFINE_ITERATIONS, true,
				sparseDepth, tracker, GrayU8.class, GrayU16.class);

		visualOdometry.setCalibration(realsense.getIntrinsics(),new DoNothingPixelTransform_F32());

		if(stream!=null) {
			registerStreams(stream);

			if(debug && streams.get(0) !=null) {
				streams.get(0).registerOverlayListener(ctx -> {
					overlayFeatures(ctx);
				});
			}
		}

		initialized_count = 0;


		realsense.registerListener(new Listener() {

			double dt; int mf=0; int fpm;
			int qual_error_count=0;

			@Override
			public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth) {

				if(!do_odometry)
					return;


				if(dt >0) {
					fpm += (int)(1f/dt+0.5f);
					if((System.currentTimeMillis() - fps_tms) > 500) {
						fps_tms = System.currentTimeMillis();
						if(mf>0)
							fps = fpm/mf;
						mf=0; fpm=0;
					}
					mf++;
				}

				try {
					ConvertImage.average(rgb, gray);
					//	ConvertImage.convert(depth, gray);

					for(IVisualStreamHandler stream : streams)
						stream.addToStream(gray, depth, model, System.currentTimeMillis()*1000);


					if( !visualOdometry.process(gray,depth,setAttitudeToState(model, current))) {
						init("Odometry");
						return;
					}
				} catch( Exception e) {
					if(debug)
						System.out.println("[vis] Odometry failure: "+e.getMessage());
					init("Exception");
				}

				quality = (int)(visualOdometry.getQuality());
				if(quality > 100) quality = 100;

				// get Measurement from odometry
				pos_raw = visualOdometry.getCameraToWorld().getT();

				// Correct camera offset to pos_raw
				pos_raw.plusIP(cam_offset_ned.T);

				if(initialized_count < INIT_COUNT) {

					// Reset for EKF2
					pos_ned.reset();

					// reset speed and old measurements
					pos_raw_old.set(0,0,0);

					if( quality > min_quality) {
						if(++initialized_count == INIT_COUNT) {
							oldTimeDepth_us = 0;

							if(debug && (System.currentTimeMillis() - last_msg) > 500) {
								last_msg = System.currentTimeMillis();
								System.out.println("[vis] Odometry init at [m]: "+MSP3DUtils.vector3D_F64ToString(pos_ned.T));
								control.writeLogMessage(new LogMessage("[vis] odometry re-init: "+last_reason,
										MAV_SEVERITY.MAV_SEVERITY_NOTICE));
							}
							error_count = 0;
						}
					}  else
						initialized_count = 0;
					return;
				}

				rot_ned.setRotation(visualOdometry.getCameraToWorld().getR());
				estTimeDepth_us = System.currentTimeMillis()*1000;

				if(oldTimeDepth_us>0)
					dt = (estTimeDepth_us - oldTimeDepth_us)/1000000f;
				oldTimeDepth_us = estTimeDepth_us;

				if(!pos_raw_old.isIdentical(0, 0, 0) && dt > 0) {

					if(quality > min_quality ) {

						// speed.T = (pos_raw - pos_raw_old ) / dt
						GeometryMath_F64.sub(pos_raw, pos_raw_old, speed_ned.T);
						speed_ned.T.scale(1d/dt);

						// Check XY speed
						if(Math.sqrt(speed_ned.getX()*speed_ned.getX()+speed_ned.getZ()*speed_ned.getZ())>MAX_SPEED) {
							init("Speed");
							return;
						}

					} else {
						if(++qual_error_count > 10) {
							qual_error_count=0;
							if(debug)
								System.out.println(timeDepth+"[vis] Quality "+quality+" < Min");
							init("Quality");
						}
						pos_raw_old.set(0,0,0);
						return;
					}

					// pos_delta.T = speed.T * dt
					pos_delta.T.set(speed_ned.T); pos_delta.T.scale(dt);

					// pos.T = pos.T + pos_delta.T
					pos_ned.T.plusIP(pos_delta.T);

					ConvertRotation3D_F64.matrixToEuler(rot_ned.R, EulerType.ZXY, visAttitude);

					if(Math.abs(visAttitude[2] - model.attitude.y) > 0.1 && model.sys.isStatus(Status.MSP_LANDED)
							&& heading_init_enabled) {
						if(debug)
							System.out.println(timeDepth+"[vis] Heading not valid");
						init("Heading div.");
						return;
					}
				}
				pos_raw_old.set(pos_raw);

				if(control!=null) {
					if(error_count < MAX_ERRORS) {
						publishPX4Vision();
						model.sys.setSensor(Status.MSP_OPCV_AVAILABILITY, true);
					}
					error_count=0;
				}

				if(detectors.size()>0 && detector_cycle_ms>0 && do_odometry) {
					if((System.currentTimeMillis() - detector_tms) > detector_cycle_ms) {
						detector_tms = System.currentTimeMillis();
						model.sys.setSensor(Status.MSP_SLAM_AVAILABILITY, true);
						for(ISLAMDetector d : detectors) {
							try {
								d.process(visualOdometry, depth, gray);
							} catch(Exception e) {
								model.sys.setSensor(Status.MSP_SLAM_AVAILABILITY, false);
								System.out.println(timeDepth+"[vis] SLAM exception: "+e.getMessage());
							}
						}
					}
				}

				publisMSPVision();
			}
		});
	}

	private void overlayFeatures(Graphics ctx) {

		AccessPointTracks3D points = (AccessPointTracks3D)visualOdometry;
		for( int i = 0; i < points.getAllTracks().size(); i++ ) {
			if(points.isInlier(i))
				ctx.drawRect((int)points.getAllTracks().get(i).x,(int)points.getAllTracks().get(i).y, 1, 1);
		}

		if(points.getAllTracks().size()==0)
			ctx.drawString("No odometry", info.width-90, 20);
		else if(quality <  min_quality)
			ctx.drawString("Low quality", info.width-85, 20);
		else
			ctx.drawString((int)fps+" fps", info.width-50, 20);

		if(!Float.isNaN(model.sys.t_armed_ms) && model.sys.isStatus(Status.MSP_ARMED))
			ctx.drawString(String.format("%.1f sec",model.sys.t_armed_ms/1000), 20, 20);


	}

	public MAVVisualPositionEstimator() {
		this(new RealSenseInfo(320,240, RealSenseInfo.MODE_RGB), null, MSPConfig.getInstance(),null);
	}

	public void registerDetector(ISLAMDetector detector) {
		if(detector_cycle_ms>0) {
			System.out.println("[vis] Vision detector registered: "+detector.getClass().getSimpleName());
			detectors.add(detector);
		}
	}

	public void registerStreams(IVisualStreamHandler stream) {
		System.out.println("[vis] Vision stream registered: "+stream.getClass().getSimpleName());
		streams.add(stream);
	}

	public void start() {
		isRunning = true;
		init("StartUp");
		if(realsense!=null)
			realsense.start();
	}

	public void stop() {
		if(isRunning) {
			realsense.stop();
			publisMSPVision();
		}
		isRunning=false;
	}

	public boolean isRunning() {
		return isRunning;
	}

	public void reset() {
		init("msp reset");
	}

	private Se3_F64 setAttitudeToState(DataModel m, Se3_F64 state) {
		if(!Float.isNaN(m.attitude.r) && !Float.isNaN(m.attitude.p))
			ConvertRotation3D_F64.eulerToMatrix(EulerType.ZXY,
					m.attitude.r,
					m.attitude.p,
					m.attitude.y,
					state.getRotation());
		return state;
	}

	@SuppressWarnings("unused")
	private Se3_F64 setPositionToState(DataModel m, Se3_F64 state) {
		if(!Float.isNaN(m.state.l_y) && !Float.isNaN(m.state.l_x)) {
			state.getTranslation().y = m.state.l_z;
			state.getTranslation().x = m.state.l_y;
			state.getTranslation().z = m.state.l_x;
		}
		return state;
	}

	private void init(String reason) {

		if(visualOdometry==null)
			return;

		this.last_publish_tms = 0;
		this.last_reason = reason;

		if(do_odometry) {
			if(++error_count > MAX_ERRORS) {
				fps=0; quality=0;
				model.sys.setSensor(Status.MSP_OPCV_AVAILABILITY, false);
			}
			setAttitudeToState(model, current);
			visualOdometry.reset(current);

			if(detectors.size()>0) {
				detector_tms = System.currentTimeMillis();
				for(ISLAMDetector d : detectors)
					d.reset(model.state.l_x, model.state.l_y, model.state.l_z);
			}
			initialized_count = 0;
		}
	}

	private void publishPX4Vision() {

		if(do_odometry && (System.currentTimeMillis()-last_publish_tms) > PUBLISH_RATE_PX4) {
			last_publish_tms = System.currentTimeMillis();

			msg_local_position_ned_cov cov = new msg_local_position_ned_cov(1,2);
			cov.time_usec = (long)estTimeDepth_us;
			if(do_xy_position) {
				cov.x = (float) pos_ned.T.z;
				cov.y = (float) pos_ned.T.x;
			} else {
				cov.covariance[0]  = 999;
				cov.covariance[9]  = 999;
			}

			if(do_z_position) {
				cov.z = (float) pos_ned.T.y;
			} else
				cov.covariance[17] = 999;

			if(do_xy_speed) {
				cov.vx = (float) speed_ned.T.z;
				cov.vy = (float) speed_ned.T.x;
			} else {
				cov.covariance[24]  = 999;
				cov.covariance[30]  = 999;
			}

			if(do_z_speed) {
				cov.vz = (float) speed_ned.T.y;
			} else
				cov.covariance[35]  = 999;

			control.sendMAVLinkMessage(cov);
		}

	}

//	private void publishPX4Vision() {
//
//		if(do_position && do_odometry && (System.currentTimeMillis()-last_pos_tms) > PUBLISH_RATE_PX4) {
//			last_pos_tms = System.currentTimeMillis();
//
//			msg_vision_position_estimate sms = new msg_vision_position_estimate(1,2);
//			sms.usec = (long)estTimeDepth_us;
//			sms.x = (float) pos_ned.T.z;
//			sms.y = (float) pos_ned.T.x;
//			sms.z = (float) pos_ned.T.y;
//			sms.roll  = (float)visAttitude[0];
//			sms.pitch = (float)visAttitude[1];
//			sms.yaw   = (float)visAttitude[2];
//			control.sendMAVLinkMessage(sms);
//		}
//
//		if(do_speed && do_odometry && (System.currentTimeMillis()-last_speed_tms) > PUBLISH_RATE_PX4) {
//			last_speed_tms = System.currentTimeMillis();
//			msg_vision_speed_estimate sse = new msg_vision_speed_estimate(1,2);
//			sse.usec = (long)estTimeDepth_us;
//			sse.x = (float) speed_ned.T.z;
//			sse.y = (float) speed_ned.T.x;
//			sse.z = (float) speed_ned.T.y;
//			sse.isValid = true;
//			control.sendMAVLinkMessage(sse);
//		}
//	}

	private void publisMSPVision() {
		if((System.currentTimeMillis()-last_msp_tms) > PUBLISH_RATE_MSP) {
			last_msp_tms = System.currentTimeMillis();
			msg_msp_vision msg = new msg_msp_vision(2,1);
			msg.x =  (float) pos_ned.T.z;
			msg.y =  (float) pos_ned.T.x;
			msg.z =  (float) pos_ned.T.y;
			msg.vx = (float) speed_ned.T.z;
			msg.vy = (float) speed_ned.T.x;
			msg.vz = (float) speed_ned.T.y;
			msg.h = MSPMathUtils.fromRad((float)visAttitude[2]);   //MSPMathUtils.fromRad((float)vis_init.getY());
			msg.p = (float)visAttitude[1];
			msg.r = (float)visAttitude[0];
			msg.quality = quality;
			msg.fps = fps;
			msg.tms = (long)estTimeDepth_us;
			msg.errors = error_count;
			if(do_xy_position && do_odometry)
				msg.flags = msg.flags | 1;
			if(do_z_position && do_odometry)
				msg.flags = msg.flags | 2;
			if(do_xy_speed && do_odometry)
				msg.flags = msg.flags | 4;
			if(do_z_speed && do_odometry)
				msg.flags = msg.flags | 8;
			msg.tms = (long)estTimeDepth_us;
			control.sendMAVLinkMessage(msg);
		}
	}

	public static void main(String[] args) {
		new MAVVisualPositionEstimator();
	}

}
