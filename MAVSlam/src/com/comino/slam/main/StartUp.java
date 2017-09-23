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

package com.comino.slam.main;

import java.io.IOException;
import java.lang.management.MemoryMXBean;
import java.lang.management.OperatingSystemMXBean;
import java.net.InetSocketAddress;

import org.mavlink.messages.lquac.msg_msp_micro_grid;
import org.mavlink.messages.lquac.msg_msp_status;

import com.comino.mav.control.IMAVMSPController;
import com.comino.mav.control.impl.MAVProxyController;
import com.comino.msp.execution.commander.MSPCommander;
import com.comino.msp.log.MSPLogger;
import com.comino.msp.main.MSPConfig;
import com.comino.msp.model.DataModel;
import com.comino.msp.utils.WifiQuality;
import com.comino.realsense.boofcv.RealSenseInfo;
import com.comino.server.mjpeg.impl.HttpMJPEGHandler;
import com.comino.slam.detectors.impl.VfhFeatureDetector;
import com.comino.slam.estimators.IPositionEstimator;
import com.comino.slam.estimators.MAVPositionEstimatorAttitude;
import com.sun.net.httpserver.HttpServer;

public class StartUp implements Runnable {

	IMAVMSPController    control = null;
	MSPConfig	          config  = null;

	private OperatingSystemMXBean osBean = null;
	private MemoryMXBean mxBean = null;

	private MSPCommander  commander = null;

	IPositionEstimator vision = null;
	private boolean publish_microslam;

	public StartUp(String[] args) {

		RealSenseInfo info = null;

		config  = MSPConfig.getInstance("/home/up","msp.properties");
		System.out.println("MSPControlService version "+config.getVersion());

		if(args.length>0)
			control = new MAVProxyController(true);
		else
			control = new MAVProxyController(false);

		osBean =  java.lang.management.ManagementFactory.getOperatingSystemMXBean();
		mxBean = java.lang.management.ManagementFactory.getMemoryMXBean();

		MSPLogger.getInstance(control);

		commander = new MSPCommander(control);

		// Start services if required

		try {
			if(config.getBoolProperty("vision_enabled", "false")) {

				if(config.getBoolProperty("vision_highres", "false"))
					info = new RealSenseInfo(640,480, RealSenseInfo.MODE_RGB);
				else
					info = new RealSenseInfo(320,240, RealSenseInfo.MODE_RGB);


				HttpMJPEGHandler streamer = new HttpMJPEGHandler(info, control.getCurrentModel());

				// Start HTTP Service with MJPEG streamer


				vision = new MAVPositionEstimatorAttitude(info, control, config, streamer);
				//			vision = new RealSensePositionEstimator(info, control, config, streamer);
				//	vision.registerDetector(new SimpleCollisionDetector(control,config,streamer));
				vision.registerDetector(new VfhFeatureDetector(control,config,streamer, commander.getOffBoardUpdater()));

				HttpServer server;
				try {
					server = HttpServer.create(new InetSocketAddress(8080),2);
					server.createContext("/mjpeg", streamer);
					server.setExecutor(null); // creates a default executor
					server.start();
				} catch (IOException e) {
					System.err.println(e.getMessage());
				}

			}
		} catch(Exception e) {
			System.out.println("[vis] Vision not available: "+e.getMessage());
		}

		//		if(config.getBoolProperty("file_stream_enabled", "false"))
		//			vision.registerStreams(new CombinedFileStreamHandler(info, control));

		this.publish_microslam = config.getBoolProperty("publish_microslam", "false");
		System.out.println("[vis] Publishing microSlam enabled: "+publish_microslam);


		if(vision!=null && !vision.isRunning()) {
			vision.start();

		}

		// Experimental circle mode disabled for safety. Test JumpBack first
//		control.addStatusChangeListener((o,n) -> {
//			if(n.isAutopilotModeChanged(o, MSP_AUTOCONTROL_MODE.CIRCLE_MODE)) {
//				commander.getOffBoardUpdater().setExperimentalCirleMode(n.isAutopilotMode(MSP_AUTOCONTROL_MODE.CIRCLE_MODE));
//
//			}
//		});

		// register MSP commands here

		control.start();

		MSPLogger.getInstance().writeLocalMsg("MAVProxy "+config.getVersion()+" loaded");
		Thread worker = new Thread(this);
		worker.start();

		//	control.getCurrentModel().slam.setBlock(-1, -3);


	}

	public static void main(String[] args) {
		new StartUp(args);

	}


	@Override
	public void run() {
		long tms = System.currentTimeMillis();
		DataModel model = control.getCurrentModel();

		WifiQuality wifi = new WifiQuality();

		while(true) {
			try {
				Thread.sleep(250);


				if(!control.isConnected()) {
					control.connect();
					continue;
				}

				wifi.getQuality();


				if(publish_microslam) {
					msg_msp_micro_grid msg = new msg_msp_micro_grid(2,1);
					msg.resolution = 0;
					msg.extension  = 0;
					msg.cx  = model.grid.getIndicatorX();
					msg.cy  = model.grid.getIndicatorY();
					msg.tms  = System.nanoTime() / 1000;
					msg.count = model.grid.count;
					if(model.grid.toArray(msg.data))
						control.sendMAVLinkMessage(msg);
				}

				msg_msp_status msg = new msg_msp_status(2,1);
				msg.load = (int)(osBean.getSystemLoadAverage()*100);
				msg.memory = (int)(mxBean.getHeapMemoryUsage().getUsed() * 100 /mxBean.getHeapMemoryUsage().getMax());
				msg.wifi_quality = (byte)wifi.get();
				msg.com_error = control.getErrorCount();
				msg.autopilot_mode =control.getCurrentModel().sys.autopilot;
				msg.uptime_ms = System.currentTimeMillis() - tms;
				msg.status = control.getCurrentModel().sys.getStatus();
				msg.setVersion(config.getVersion());
				msg.setArch(osBean.getArch());
				msg.unix_time_us = System.currentTimeMillis() * 1000;
				control.sendMAVLinkMessage(msg);

			} catch (Exception e) {
				e.printStackTrace();
				control.close();
			}
		}
	}
}
