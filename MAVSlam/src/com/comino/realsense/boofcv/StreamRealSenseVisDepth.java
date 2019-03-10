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


package com.comino.realsense.boofcv;

import java.util.ArrayList;
import java.util.List;

import com.comino.librealsense.wrapper.LibRealSenseIntrinsics;
import com.comino.librealsense.wrapper.LibRealSenseUtils;
import com.comino.librealsense.wrapper.LibRealSenseWrapper;
import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_format;
import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_intrinsics;
import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_option;
import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_stream;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;

import boofcv.struct.calib.IntrinsicParameters;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

public class StreamRealSenseVisDepth {

	private static final long MAX_RATE = 20;

	// time out used in some places
	private long timeout=10000;


	private List<Listener> listeners;

	// image with depth information
	private GrayU16 depth = new GrayU16(1,1);

	private Planar<GrayU8> rgb	 = new Planar<GrayU8>(GrayU8.class,1,1,3);



	private CombineThread thread;

	private PointerByReference error= new PointerByReference();
	private PointerByReference ctx;

	private PointerByReference dev;

	private RealSenseInfo info;
	private float scale;

	private LibRealSenseIntrinsics intrinsics;

	public StreamRealSenseVisDepth(int devno , RealSenseInfo info)
	{

		ctx = LibRealSenseWrapper.INSTANCE.rs_create_context(11201, error);
		if(LibRealSenseWrapper.INSTANCE.rs_get_device_count(ctx, error)<1)
			ctx = LibRealSenseWrapper.INSTANCE.rs_create_context(4, error);

		if(LibRealSenseWrapper.INSTANCE.rs_get_device_count(ctx, error)<1) {
			LibRealSenseWrapper.INSTANCE.rs_delete_context(ctx, error);
			throw new IllegalArgumentException("No device found");
		}
		this.listeners = new ArrayList<Listener>();

		this.info = info;

		dev = LibRealSenseWrapper.INSTANCE.rs_get_device(ctx, devno, error);
		LibRealSenseWrapper.INSTANCE.rs_wait_for_frames(dev, error);


		Pointer ch = LibRealSenseWrapper.INSTANCE.rs_get_device_firmware_version(dev, error);
		System.out.println("Firmware version: "+ch.getString(0));

		LibRealSenseUtils.rs_apply_depth_control_preset(dev, LibRealSenseUtils.PRESET_DEPTH_HIGH);
		LibRealSenseWrapper.INSTANCE.rs_set_device_option(dev, rs_option.RS_OPTION_COLOR_ENABLE_AUTO_WHITE_BALANCE, 0, error);
		LibRealSenseWrapper.INSTANCE.rs_set_device_option(dev, rs_option.RS_OPTION_R200_EMITTER_ENABLED, 1, error);
		LibRealSenseWrapper.INSTANCE.rs_set_device_option(dev, rs_option.RS_OPTION_COLOR_ENABLE_AUTO_EXPOSURE, 1, error);
		LibRealSenseWrapper.INSTANCE.rs_set_device_option(dev, rs_option.RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED,1 , error);
		LibRealSenseWrapper.INSTANCE.rs_set_device_option(dev, rs_option.RS_OPTION_COLOR_BACKLIGHT_COMPENSATION, 0, error);


		LibRealSenseWrapper.INSTANCE.rs_enable_stream(dev, rs_stream.RS_STREAM_COLOR,
				info.width,info.height,rs_format.RS_FORMAT_RGB8, info.framerate, error);

		if(info.mode==RealSenseInfo.MODE_INFRARED) {
			LibRealSenseWrapper.INSTANCE.rs_enable_stream(dev, rs_stream.RS_STREAM_INFRARED2_ALIGNED_TO_DEPTH,
					info.width,info.height,rs_format.RS_FORMAT_ANY, info.framerate, error);
		}

		LibRealSenseWrapper.INSTANCE.rs_enable_stream(dev, rs_stream.RS_STREAM_DEPTH,
				info.width,info.height,rs_format.RS_FORMAT_Z16, info.framerate, error);


		scale = LibRealSenseWrapper.INSTANCE.rs_get_device_depth_scale(dev, error);

		rs_intrinsics rs_int= new rs_intrinsics();
		LibRealSenseWrapper.INSTANCE.rs_get_stream_intrinsics(dev, rs_stream.RS_STREAM_RECTIFIED_COLOR, rs_int, error);
		intrinsics = new LibRealSenseIntrinsics(rs_int);

		System.out.println("Depth scale: "+scale+" Intrinsics: "+intrinsics.toString());

		depth.reshape(info.width,info.height);
		rgb.reshape(info.width,info.height);

	}

	public StreamRealSenseVisDepth registerListener(Listener listener) {
		listeners.add(listener);
		return this;
	}


	public void start() {
		LibRealSenseWrapper.INSTANCE.rs_start_device(dev, error);

		thread = new CombineThread();
		thread.start();
		thread.setName("VIO");
		thread.setPriority(Thread.NORM_PRIORITY);

		while(!thread.running)
			Thread.yield();
	}

	public void stop() {
		thread.requestStop = true;
		long start = System.currentTimeMillis()+timeout;
		while( start > System.currentTimeMillis() && thread.running )
			Thread.yield();
		LibRealSenseWrapper.INSTANCE.rs_stop_device(dev, error);
	}



	public IntrinsicParameters getIntrinsics() {
		return intrinsics;
	}


	private class CombineThread extends Thread {

		public volatile boolean running = false;
		public volatile boolean requestStop = false;

		public volatile Pointer depthData;
		public volatile Pointer rgbData;

		private long timeDepth;
		private long timeRgb;

		private long tms_offset_depth  = 0;
		private long tms_offset_rgb    = 0;

		@Override
		public void run() {
			running = true; long tms; long wait;

			while( !requestStop ) {

				try {

					tms = System.currentTimeMillis();

					LibRealSenseWrapper.INSTANCE.rs_wait_for_frames(dev, error);

					depthData = LibRealSenseWrapper.INSTANCE.rs_get_frame_data(dev,
							rs_stream.RS_STREAM_DEPTH_ALIGNED_TO_RECTIFIED_COLOR, error);
					timeDepth = (long)LibRealSenseWrapper.INSTANCE.rs_get_frame_timestamp(dev,
							rs_stream.RS_STREAM_DEPTH_ALIGNED_TO_RECTIFIED_COLOR, error);
					if(tms_offset_depth==0)
						tms_offset_depth = System.currentTimeMillis() - timeDepth;

					if(depthData!=null)
						bufferDepthToU16(depthData,depth);

					switch(info.mode) {
					case RealSenseInfo.MODE_RGB:
						synchronized ( this ) {
							timeRgb = (long)LibRealSenseWrapper.INSTANCE.rs_get_frame_timestamp(dev,
									rs_stream.RS_STREAM_RECTIFIED_COLOR, error);
							if(tms_offset_rgb==0)
								tms_offset_rgb = System.currentTimeMillis() - timeRgb;
							rgbData = LibRealSenseWrapper.INSTANCE.rs_get_frame_data(dev,
									rs_stream.RS_STREAM_RECTIFIED_COLOR, error);
							if(rgbData!=null)
								bufferRgbToMsU8(rgbData,rgb);
						}
						break;
					case RealSenseInfo.MODE_INFRARED:
						synchronized ( this ) {
							timeRgb = (long)LibRealSenseWrapper.INSTANCE.rs_get_frame_timestamp(dev,
									rs_stream.RS_STREAM_INFRARED2_ALIGNED_TO_DEPTH, error);
							if(tms_offset_rgb==0)
								tms_offset_rgb = System.currentTimeMillis() - timeRgb;
							rgbData = LibRealSenseWrapper.INSTANCE.rs_get_frame_data(dev,
									rs_stream.RS_STREAM_INFRARED2_ALIGNED_TO_DEPTH, error);
							if(rgbData!=null)
								bufferGrayToMsU8(rgbData,rgb);
						}
						break;
					}

					if(listeners.size()>0) {
						for(Listener listener : listeners)
							listener.process(rgb, depth, timeRgb+tms_offset_rgb, timeDepth+tms_offset_depth);
					}

					// Limit maximum frame rate to MAX_RATE
					wait = MAX_RATE - ( System.currentTimeMillis() - tms + 1 );
					if(wait>0)
					   Thread.sleep(wait);

				} catch(Exception e) {
					e.printStackTrace();
				}
			}

			running = false;
		}
	}

	public void bufferGrayToU8(Pointer input , GrayU8 output ) {
		byte[] inp = input.getByteArray(0, output.width * output.height );
		System.arraycopy(inp, 0, output.data, 0, output.width * output.height);
	}


	public void bufferDepthToU16(Pointer input , GrayU16 output ) {
		short[] inp = input.getShortArray(0, output.width * output.height);
		System.arraycopy(inp, 0, output.data, 0, output.width * output.height);
	}


	private int x,y,indexOut, indexIn;
	private byte[] input;

	public void bufferRgbToMsU8( Pointer inp , Planar<GrayU8> output ) {

		input = inp.getByteArray(0, output.width * output.height * 3);

		indexIn = 0;
		for( y = 0; y < output.height; y++ ) {
			indexOut = output.startIndex + y*output.stride;
			for( x = 0; x < output.width; x++ , indexOut++ ) {
				output.getBand(0).data[indexOut] = input[indexIn++];
				output.getBand(1).data[indexOut] = input[indexIn++];
				output.getBand(2).data[indexOut] = input[indexIn++];
			}
		}
	}

	public void bufferGrayToMsU8( Pointer inp , Planar<GrayU8> output ) {

		input = inp.getByteArray(0, output.width * output.height);

		indexIn = 0;
		for(y = 0; y < output.height; y++ ) {
			int indexOut = output.startIndex + y*output.stride;
			for(x = 0; x < output.width; x++ , indexOut++ ) {
				output.getBand(0).data[indexOut] = input[indexIn];
				output.getBand(1).data[indexOut] = input[indexIn];
				output.getBand(2).data[indexOut] = input[indexIn++];
			}
		}
	}

	public interface Listener {
		public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth);
	}
}
