package com.comino.slam.main;

import java.lang.management.MemoryMXBean;
import java.lang.management.OperatingSystemMXBean;

import org.mavlink.messages.lquac.msg_msp_command;
import org.mavlink.messages.lquac.msg_msp_status;

import com.comino.mav.control.IMAVMSPController;
import com.comino.mav.control.impl.MAVProxyController;
import com.comino.msp.log.MSPLogger;
import com.comino.msp.main.MSPConfig;
import com.comino.msp.main.control.listener.IMAVLinkListener;
import com.comino.msp.model.segment.Status;
import com.comino.realsense.boofcv.RealSensePositionEstimator;

public class StartUp implements Runnable {

	IMAVMSPController    control = null;
	MSPConfig	          config  = null;

	private OperatingSystemMXBean osBean = null;
	private MemoryMXBean mxBean = null;

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


		// TODO 1.0: Start services if required

		try {
		  if(config.getBoolProperty("vision_enabled", "true"))
		     vision = new RealSensePositionEstimator(control);
		} catch(Exception e) {
			System.out.println("Vision not available: "+e.getMessage());
		}

		// TODO 1.0: register MSP commands here

		control.registerListener(msg_msp_command.class, new IMAVLinkListener() {
			@Override
			public void received(Object o) {
				msg_msp_command hud = (msg_msp_command)o;
				MSPLogger.getInstance().writeLocalMsg("Companion Command "+hud.command+" executed");
			}
		});

		control.start();

		control.connect();
		MSPLogger.getInstance().writeLocalMsg("MAVProxy "+config.getVersion()+" loaded");
        Thread worker = new Thread(this);
        worker.start();

        control.getCurrentModel().sys.setMSPStatus(Status.MSP_HEALTH_OK, true);
	}



	public static void main(String[] args) {
		new StartUp(args);

	}



	@Override
	public void run() {
		long tms = System.currentTimeMillis();
		while(true) {
			try {
				Thread.sleep(2000);

				if(vision!=null && !vision.isRunning())
					vision.start();

				if(!control.isConnected()) {
					control.connect();
					continue;
				}

				msg_msp_status msg = new msg_msp_status(1,2);
				msg.load = (int)(osBean.getSystemLoadAverage()*100);
				msg.memory = (int)(mxBean.getHeapMemoryUsage().getUsed() * 100 /mxBean.getHeapMemoryUsage().getMax());
				msg.com_error = control.getErrorCount();
				msg.uptime_ms = System.currentTimeMillis() - tms;
				msg.status = control.getCurrentModel().sys.msp_status;
				msg.setVersion(config.getVersion());
				msg.setArch(osBean.getArch());
				control.sendMAVLinkMessage(msg);

			} catch (Exception e) {
				control.close();
			}
		}

	}

}
