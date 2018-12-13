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

package com.comino.slam.estimators.vio;


import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;
import java.util.List;

import org.mavlink.messages.MAV_SEVERITY;
import org.mavlink.messages.MSP_CMD;
import org.mavlink.messages.MSP_COMPONENT_CTRL;
import org.mavlink.messages.lquac.msg_msp_command;
import org.mavlink.messages.lquac.msg_msp_vision;
import org.mavlink.messages.lquac.msg_vision_position_estimate;

import com.comino.main.MSPConfig;
import com.comino.mav.control.IMAVMSPController;
import com.comino.msp.execution.control.listener.IMAVLinkListener;
import com.comino.msp.log.MSPLogger;
import com.comino.msp.model.DataModel;
import com.comino.msp.model.segment.Status;
import com.comino.msp.utils.ExecutorService;
import com.comino.msp.utils.MSPMathUtils;
import com.comino.realsense.boofcv.RealSenseInfo;
import com.comino.realsense.boofcv.StreamRealSenseVisDepth;
import com.comino.realsense.boofcv.StreamRealSenseVisDepth.Listener;
import com.comino.server.mjpeg.IVisualStreamHandler;
import com.comino.slam.boofcv.MAVDepthVisualOdometry;
import com.comino.slam.boofcv.vio.FactoryMAVOdometryVIO;
import com.comino.slam.boofcv.vio.tracker.FactoryMAVPointTrackerTwoPassVIO;
import com.comino.slam.detectors.ISLAMDetector;
import com.comino.slam.estimators.IPositionEstimator;

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
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;

public class MAVVisualPositionEstimatorVIO implements IPositionEstimator {

	private static final int   	PUBLISH_RATE_MSP	    = 50 - 5;
	private static final int  	PUBLISH_RATE_PX4    	= 10 - 5;

	private static final int    INIT_COUNT           	= 2;
	private static final int    MAX_ERRORS    	    	= 5;
	private static final int    MAX_QUALITY_ERRORS   	= 10;

	private static final int    MAX_SPEED    	    	= 20;
	private static final float  VISION_POS_GATE     	= 0.25f;
	private static final float  VISION_SPEED_GATE     	= 0.25f;

	private static final float  INLIER_PIXEL_TOL    	= 1.5f;
	private static final int    MAXTRACKS   			= 180;
	private static final int    KLT_RADIUS          	= 3;
	private static final float  KLT_THRESHOLD       	= 1f;
	private static final int    RANSAC_ITERATIONS   	= 150;
	private static final int    RETIRE_THRESHOLD    	= 2;
	private static final int    ADD_THRESHOLD       	= 50;
	private static final int    REFINE_ITERATIONS   	= 60;

	private static final int    MIN_MESSAGE_INTERVAL_MS = 500;

	// TODO: get mounting offset of camera from config file
	private final Point3D_F64 mounting_offset = new Point3D_F64(0.015,-0.057,0.068);


	private StreamRealSenseVisDepth 				    realsense			= null;
	private MAVDepthVisualOdometry<GrayU8,GrayU16>    	visualOdometry		= null;
	private RealSenseInfo 						    	info				= null;

	private GrayU8 gray 			= null;

	private double oldTimeDepth_us	= 0;
	private double estTimeDepth_us	= 0;


	//	private Quaternion_F64 att_q	= new Quaternion_F64();
	private double[] visAttitude     = new double[3];

	private long last_pos_tms        = 0;
	private long last_msp_tms        = 0;

	private DataModel model;

	private boolean debug 					= false;
	private boolean heading_init_enabled 	= false;
	private boolean isRunning    			= false;

	private Se3_F64 pose                    = new Se3_F64();
	private Se3_F64 pose_old                = new Se3_F64();
	private Se3_F64 speed                   = new Se3_F64();

	private int quality				= 0;
	private int min_quality 		= 0;

	private long detector_tms 		= 0;
	private int  detector_cycle_ms 	= 250;

	private float vision_pos_gate    = 0;
	private float vision_speed_gate  = 0;

	private float fps 				= 0;
	private long  fps_tms         	= 0;

	private long publish_tms_us     = 0;


	private int initialized_count  	= 0;
	private int error_count 		= 0;
	private long last_msg_tms       = 0;

	private boolean do_odometry 	= true;
	private boolean do_xy_position 	= false;
	private boolean do_xy_speed 	= false;
	private boolean do_attitude		= false;
	private boolean do_covariances  = false;

	private IMAVMSPController 							control		= null;
	private List<ISLAMDetector> 						detectors 	= null;
	private List<IVisualStreamHandler<Planar<GrayU8>>>	streams 	= null;


	private final Color	bgColor = new Color(128,128,128,130);


	public <T> MAVVisualPositionEstimatorVIO(RealSenseInfo info, IMAVMSPController control, MSPConfig config, IVisualStreamHandler<T> stream) {

		this.info    = info;
		this.control = control;
		this.detectors = new ArrayList<ISLAMDetector>();
		this.streams   = new ArrayList<IVisualStreamHandler<Planar<GrayU8>>>();

		System.out.println("Vision position estimator: "+this.getClass().getSimpleName());
		this.debug = config.getBoolProperty("vision_debug", "true");
		this.heading_init_enabled = config.getBoolProperty("vision_heading_init", "true");
		System.out.println("Vision debugging: "+debug);
		System.out.println("Initialize heading when landed: "+heading_init_enabled);
		System.out.println("Vision setup: MaxTracks="+MAXTRACKS+" RanSac="+RANSAC_ITERATIONS+ " KLTRadius="+KLT_RADIUS+ " KLTThreshold="+KLT_THRESHOLD);
		this.min_quality = config.getIntProperty("vision_min_quality", "10");
		System.out.println("Vision minimum quality: "+min_quality);

		this.vision_pos_gate = config.getFloatProperty("vision_pos_gate", String.valueOf(VISION_POS_GATE));
		System.out.println("Vision position gate: "+vision_pos_gate+"m");

		this.vision_speed_gate = config.getFloatProperty("vision_speed_gate", String.valueOf(VISION_SPEED_GATE));
		System.out.println("Vision speed gate: "+vision_speed_gate+"m/s");

		this.do_odometry = config.getBoolProperty("vision_enable", "true");
		System.out.println("Vision Odometry enabled: "+do_odometry);

		this.do_xy_position = config.getBoolProperty("vision_pub_pos_xy", "true");
		System.out.println("Vision publishes XY position: "+do_xy_position);

		this.do_xy_speed = config.getBoolProperty("vision_pub_speed_xy", "true");
		System.out.println("Vision publishes XY speed: "+do_xy_speed);

		this.do_attitude = config.getBoolProperty("vision_pub_attitude", "true");
		System.out.println("Vision publishes attitude: "+do_attitude);

		this.do_covariances = config.getBoolProperty("vision_pub_covariance", "true");
		System.out.println("Vision publishes covariances: "+do_covariances);

		this.detector_cycle_ms = config.getIntProperty("vision_detector_cycle", "100");
		if(this.detector_cycle_ms > 0)
			System.out.printf("Vision detectors enablied with %d [ms] cycle \n",detector_cycle_ms);

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

		//reset vision when GPOS gets valid
		control.getStatusManager().addListener(Status.MSP_GPOS_VALID, (o,n) -> {
			if((n.isStatus(Status.MSP_GPOS_VALID)))
				reset();
		});

		try {
			realsense = new StreamRealSenseVisDepth(0,info);
		} catch(Exception e) {
			this.do_odometry = false;
			this.detector_cycle_ms = 0;
			System.out.println("Odometry disabled ("+e.getMessage()+").");
			return;
		}

		PkltConfig configKlt = new PkltConfig();
		configKlt.pyramidScaling = new int[]{ 1, 2, 4, 8 };
		configKlt.templateRadius = 3;

		PointTrackerTwoPass<GrayU8> tracker =
				FactoryMAVPointTrackerTwoPassVIO.klt(configKlt, new ConfigGeneralDetector(MAXTRACKS, KLT_RADIUS, KLT_THRESHOLD),
						GrayU8.class, GrayS16.class);

		DepthSparse3D<GrayU16> sparseDepth = new DepthSparse3D.I<GrayU16>(1e-3);


		visualOdometry = FactoryMAVOdometryVIO.depthPnP(INLIER_PIXEL_TOL,
				ADD_THRESHOLD, RETIRE_THRESHOLD, RANSAC_ITERATIONS, REFINE_ITERATIONS, true, mounting_offset,
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

				publish_tms_us = System.currentTimeMillis()*1000;

				for(IVisualStreamHandler<Planar<GrayU8>> stream : streams)
					stream.addToStream(rgb, model, System.currentTimeMillis()*1000);

				if(!do_odometry || visualOdometry == null ) {
					return;
				}

				// Averaging frames per second
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


					if(control.isSimulation()) {
						if( !visualOdometry.process(gray,depth,null)) {
							init("Tracking");
							return;
						}
					} else {
						setModelToState(model, pose);
						if( !visualOdometry.process(gray,depth,pose)) {
							init("Tracking");
							return;
						}
					}

				} catch( Exception e) {
					if(debug)
						System.out.println("[vis] Odometry failure: "+e.getMessage());
					pose_old.set(pose);
					init("Exception");
					return;
				}

				quality = (int)((visualOdometry.getQuality())*100f / (MAXTRACKS));
				if(quality > 100) quality = 100; if(quality < 1) quality = 1;


				// get Measurement from odometry
				pose.set(visualOdometry.getCameraToWorld());


				estTimeDepth_us = timeDepth * 1000d;
				//	estTimeDepth_us = publish_tms_us;
				dt = (estTimeDepth_us - oldTimeDepth_us)/1000000d;

				if(oldTimeDepth_us==0) {
					oldTimeDepth_us = estTimeDepth_us;
					return;
				}
				oldTimeDepth_us = estTimeDepth_us;

				// do some initializing measurements first
				if(initialized_count++ < INIT_COUNT) {
					pose_old.set(pose);
					return;
				}

				if(dt > 0) {

					if(quality > min_quality ) {

						qual_error_count=0;

						// speed.T = (pos_raw - pos_raw_old ) / dt
						GeometryMath_F64.sub(pose.T, pose_old.T, speed.T);
						speed.T.scale(1d/dt);


						// Check XY speed
						if(Math.sqrt(speed.getX()*speed.getX()+speed.getZ()*speed.getZ())>MAX_SPEED) {
							publisMSPVision();
							init("Speed");
							return;
						}

					} else {

						if(++qual_error_count > MAX_QUALITY_ERRORS) {
							publisMSPVision();
							init("Quality");
						}
						return;
					}

					ConvertRotation3D_F64.matrixToEuler(pose.R, EulerType.ZXY, visAttitude);

					//ConvertRotation3D_F64.eulerToQuaternion(EulerType.XYZ,visAttitude[0],visAttitude[1], visAttitude[2], att_q);


					if(Math.abs(visAttitude[2] - model.attitude.y) > 0.1 && model.sys.isStatus(Status.MSP_LANDED)
							&& heading_init_enabled && !control.isSimulation()) {
						init("Heading div.");
						return;
					}
				}
				pose_old.set(pose);


				if(	( Math.abs(pose.T.z - model.state.l_x) > vision_pos_gate ||
						Math.abs(pose.T.x - model.state.l_y) > vision_pos_gate ) && !control.isSimulation())   {
					pose_old.set(pose);
					init("Vision pos. gate");
					return;
				}

				publishPX4Vision();
				error_count=0;

				if(detectors.size()>0 && detector_cycle_ms>0 && do_odometry) {
					if((System.currentTimeMillis() - detector_tms) > detector_cycle_ms) {
						detector_tms = System.currentTimeMillis();
						model.sys.setSensor(Status.MSP_SLAM_AVAILABILITY, true);
						ExecutorService.submit(() -> {
							for(ISLAMDetector d : detectors) {
								try {
									d.process(visualOdometry, depth, gray);
								} catch(Exception e) {
									model.sys.setSensor(Status.MSP_SLAM_AVAILABILITY, false);
									//System.out.println(timeDepth+"[vis] SLAM exception: "+e.getMessage());
								}
							}
						}, ExecutorService.LOW);
					}
				}

				// Publish MSP data
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

		ctx.setColor(bgColor);
		ctx.fillRect(5, 5, info.width-10, 21);
		ctx.setColor(Color.white);

		if(points.getAllTracks().size()==0)
			ctx.drawString("No odometry", info.width-90, 20);
		else if(quality <  min_quality)
			ctx.drawString("Low quality", info.width-85, 20);
		else
			ctx.drawString((int)fps+" fps", info.width-50, 20);

		if(!Float.isNaN(model.sys.t_armed_ms) && model.sys.isStatus(Status.MSP_ARMED))
			ctx.drawString(String.format("%.1f sec",model.sys.t_armed_ms/1000), 20, 20);

		if(model.msg.text != null && (model.sys.getSynchronizedPX4Time_us()-model.msg.tms) < 1000000)
			ctx.drawString(model.msg.text, 10, info.height-5);

	}

	public MAVVisualPositionEstimatorVIO() {
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


	private Se3_F64 setModelToState(DataModel m, Se3_F64 state) {
		if(!Float.isNaN(m.attitude.r) && !Float.isNaN(m.attitude.p))
			ConvertRotation3D_F64.eulerToMatrix(EulerType.ZXY,
					m.attitude.r,
					m.attitude.p,
					m.attitude.y,
					state.getRotation());

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

		this.initialized_count = 0;

		this.last_pos_tms = 0;
		visualOdometry.reset();

		if(do_odometry) {

			if((System.currentTimeMillis()-last_msg_tms)>MIN_MESSAGE_INTERVAL_MS && error_count < MAX_ERRORS) {
				  MSPLogger.getInstance().writeLocalMsg("[vio] Init ("+reason+")",
						MAV_SEVERITY.MAV_SEVERITY_WARNING);
				  last_msg_tms = System.currentTimeMillis();
				}

			if(++error_count > MAX_ERRORS) {
				fps=0; quality=0;
				model.sys.setSensor(Status.MSP_OPCV_AVAILABILITY, false);
			}

			if(detectors.size()>0) {
				detector_tms = System.currentTimeMillis();
				for(ISLAMDetector d : detectors)
					d.reset(model.state.l_x, model.state.l_y, model.state.l_z);
			}

		}
	}



	private void publishPX4Vision() {
		if(do_odometry && (System.currentTimeMillis()-last_pos_tms) > PUBLISH_RATE_PX4) {
			last_pos_tms = System.currentTimeMillis();

			msg_vision_position_estimate sms = new msg_vision_position_estimate(1,2);
			//		sms.usec = (long)estTimeDepth_us;
			sms.usec = (long)publish_tms_us;
			if(do_xy_position)  {
				sms.x = (float) pose.T.z;
				sms.y = (float) pose.T.x;
				sms.z = (float) pose.T.y;
			} else {
				sms.x = Float.NaN;
				sms.y = Float.NaN;
				sms.z = Float.NaN;
			}


			if(do_attitude) {
				sms.roll  = (float)visAttitude[0];
				sms.pitch = (float)visAttitude[1];
				sms.yaw   = (float)visAttitude[2];
			} else {
				sms.roll  = Float.NaN;
				sms.pitch = Float.NaN;
				sms.yaw   = Float.NaN;
			}

			sms.covariance[0] = Float.NaN;

			control.sendMAVLinkMessage(sms);

			model.sys.setSensor(Status.MSP_OPCV_AVAILABILITY, true);

		}
	}

	private void publisMSPVision() {
		if((System.currentTimeMillis()-last_msp_tms) > PUBLISH_RATE_MSP) {
			last_msp_tms = System.currentTimeMillis();
			msg_msp_vision msg = new msg_msp_vision(2,1);
			msg.x =  (float) pose.T.z;
			msg.y =  (float) pose.T.x;
			msg.z =  (float) pose.T.y;
			msg.vx = (float) speed.T.z;
			msg.vy = (float) speed.T.x;
			msg.vz = (float) speed.T.y;
			msg.h = MSPMathUtils.fromRad((float)visAttitude[2]);   //MSPMathUtils.fromRad((float)vis_init.getY());
			msg.p = (float)visAttitude[1];
			msg.r = (float)visAttitude[0];

			msg.quality = quality;
			msg.fps = fps;
			//	msg.tms = (long)estTimeDepth_us;
			msg.tms = publish_tms_us;
			msg.errors = error_count;
			if(do_xy_position && do_odometry)
				msg.flags = msg.flags | 1;
			if(do_xy_speed && do_odometry)
				msg.flags = msg.flags | 2;
			if(do_attitude && do_odometry)
				msg.flags = msg.flags | 4;
			msg.tms = (long)estTimeDepth_us;
			control.sendMAVLinkMessage(msg);
		}
	}

}
