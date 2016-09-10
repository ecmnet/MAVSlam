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

package com.comino.slam.main;

import java.lang.management.MemoryMXBean;
import java.lang.management.OperatingSystemMXBean;

import org.mavlink.messages.lquac.msg_msp_status;

import com.comino.mav.control.IMAVMSPController;
import com.comino.mav.control.impl.MAVProxyController;
import com.comino.msp.log.MSPLogger;
import com.comino.msp.main.MSPConfig;
import com.comino.msp.main.commander.MSPCommander;
import com.comino.slam.estimator.RealSensePositionEstimator;

public class StartUp implements Runnable {

	IMAVMSPController    control = null;
	MSPConfig	          config  = null;

	private OperatingSystemMXBean osBean = null;
	private MemoryMXBean mxBean = null;

	private MSPCommander  commander = null;

	RealSensePositionEstimator vision = null;

	public StartUp(String[] args) {

		config  = MSPConfig.getInstance("msp.properties");
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
		  if(config.getBoolProperty("vision_enabled", "true"))
		     vision = new RealSensePositionEstimator(control, config.getBoolProperty("vision_debug", "false"));
		} catch(Exception e) {
			System.out.println("Vision not available: "+e.getMessage());
		}

		// register MSP commands here

		control.start();

		control.connect();
		MSPLogger.getInstance().writeLocalMsg("MAVProxy "+config.getVersion()+" loaded");
        Thread worker = new Thread(this);
        worker.start();

	}



	public static void main(String[] args) {
		new StartUp(args);

	}



	@Override
	public void run() {
		long tms = System.currentTimeMillis();

		if(vision!=null && !vision.isRunning())
			vision.start();

		while(true) {
			try {
				Thread.sleep(2000);


				if(!control.isConnected()) {
					control.connect();
					continue;
				}

				msg_msp_status msg = new msg_msp_status(1,2);
				msg.load = (int)(osBean.getSystemLoadAverage()*100);
				msg.memory = (int)(mxBean.getHeapMemoryUsage().getUsed() * 100 /mxBean.getHeapMemoryUsage().getMax());
				msg.com_error = control.getErrorCount();
				msg.uptime_ms = System.currentTimeMillis() - tms;
				msg.status = control.getCurrentModel().sys.getStatus();
				msg.setVersion(config.getVersion());
				msg.setArch(osBean.getArch());
				control.sendMAVLinkMessage(msg);

			} catch (Exception e) {
				control.close();
			}
		}

	}

}
