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


package com.comino.realsense.boofcv;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;
import boofcv.struct.image.Planar;

import com.comino.librealsense.wrapper.LibRealSenseWrapper;
import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_format;
import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_preset;
import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_stream;
import com.sun.jna.Pointer;
import com.sun.jna.ptr.PointerByReference;

public class StreamRealSenseRgbDepth {

	// time out used in some places
	private long timeout=10000;


	private Listener listener;

	// image with depth information
	private GrayU16 depth = new GrayU16(1,1);
	// image with color information
	private Planar<GrayU8> rgb = new Planar<GrayU8>(GrayU8.class,1,1,3);

	// thread which synchronized video streams
	private CombineThread thread;

	private PointerByReference error= new PointerByReference();
	private PointerByReference ctx;

	PointerByReference dev;


	public void start(int devno , RealSenseInfo info , Listener listener )
	{
		ctx = LibRealSenseWrapper.INSTANCE.rs_create_context(4, error);

		if(LibRealSenseWrapper.INSTANCE.rs_get_device_count(ctx, error)<1) {
			throw new IllegalArgumentException("No device connected");
		}

		this.listener = listener;

		dev = LibRealSenseWrapper.INSTANCE.rs_get_device(ctx, devno, error);
		Pointer ch = LibRealSenseWrapper.INSTANCE.rs_get_device_firmware_version(dev, error);
		System.out.println("Firmware version: "+ch.getString(0));

		// 480x360 seems not to be supported in RGB but in DEPTH
		LibRealSenseWrapper.INSTANCE.rs_enable_stream(dev, rs_stream.RS_STREAM_COLOR,
				640, 480,rs_format.RS_FORMAT_RGB8, info.framerate, error);
		LibRealSenseWrapper.INSTANCE.rs_enable_stream_preset(dev, rs_stream.RS_STREAM_DEPTH,
				rs_preset.RS_PRESET_BEST_QUALITY, error);

		//				LibRealSenseWrapper.INSTANCE.rs_enable_stream(dev, rs_stream.RS_STREAM_DEPTH,
		//						  info.width, info.height,rs_format.RS_FORMAT_Z16, info.framerate, error);

		//		LibRealSenseWrapper.INSTANCE.rs_enable_stream(dev, rs_stream.RS_STREAM_DEPTH,
		//				  info.width, info.height,rs_format.RS_FORMAT_Z16, info.framerate, error);
		//		LibRealSenseWrapper.INSTANCE.rs_enable_stream_preset(dev, rs_stream.RS_STREAM_COLOR,
		//				rs_preset.RS_PRESET_BEST_QUALITY, error);



		depth.reshape(info.width,info.height);
		rgb.reshape(info.width,info.height);

		LibRealSenseWrapper.INSTANCE.rs_start_device(dev, error);

		thread = new CombineThread();
		thread.start();
		// make sure the thread is running before moving on
		while(!thread.running)
			Thread.yield();


	}

	/**
	 * Stops all the threads from running and closes the video channels and video device
	 */
	public void stop() {
		thread.requestStop = true;
		long start = System.currentTimeMillis()+timeout;
		while( start > System.currentTimeMillis() && thread.running )
			Thread.yield();

		LibRealSenseWrapper.INSTANCE.rs_stop_device(dev, error);
	}


	private class CombineThread extends Thread {

		public volatile boolean running = false;
		public volatile boolean requestStop = false;

		public volatile Pointer depthData;
		public volatile Pointer rgbData;

		@Override
		public void run() {
			running = true;
			long timeDepth = 0,timeRgb = 0;

			while( !requestStop ) {

				LibRealSenseWrapper.INSTANCE.rs_wait_for_frames(dev, error);

				synchronized (this ) {
					timeDepth = LibRealSenseWrapper.INSTANCE.rs_get_frame_timestamp(dev,
							rs_stream.RS_STREAM_DEPTH, error);
					depthData = LibRealSenseWrapper.INSTANCE.rs_get_frame_data(dev,
							rs_stream.RS_STREAM_DEPTH, error);
					bufferDepthToU16(depthData,depth);
				}
				synchronized ( this ) {
					timeRgb = LibRealSenseWrapper.INSTANCE.rs_get_frame_timestamp(dev,
							rs_stream.RS_STREAM_COLOR, error);
					rgbData = LibRealSenseWrapper.INSTANCE.rs_get_frame_data(dev,
							rs_stream.RS_STREAM_COLOR, error);
					bufferRgbToMsU8(rgbData,rgb);
				}

				listener.process(rgb, depth, timeRgb, timeDepth);
			}

			running = false;
		}
	}


	public void bufferDepthToU16(Pointer input , GrayU16 output ) {
		output.setData(input.getShortArray(0, output.width * output.height));
	}

	public void bufferRgbToMsU8( Pointer inp , Planar<GrayU8> output ) {
		byte[] input = inp.getByteArray(0, output.width * output.height * 3);
		GrayU8 band0 = output.getBand(0);
		GrayU8 band1 = output.getBand(1);
		GrayU8 band2 = output.getBand(2);

		int indexIn = 0;
		for( int y = 0; y < output.height; y++ ) {
			int indexOut = output.startIndex + y*output.stride;
			for( int x = 0; x < output.width; x++ , indexOut++ ) {
				band0.data[indexOut] = input[indexIn++];
				band1.data[indexOut] = input[indexIn++];
				band2.data[indexOut] = input[indexIn++];
			}
		}
	}

	/**
	 * Listener for kinect data
	 */
	public interface Listener {
		/**
		 * Function for processing synchronized kinect data. The two most recent depth and rgb images are passed along
		 * with their time stamps.  The user can spend as much time inside this function without screwing up the
		 * video feeds.  Just make sure you exit it before calling stop.
		 * @param rgb Color image
		 * @param depth Depth image
		 * @param timeRgb Time-stamp for rgb image
		 * @param timeDepth Time-stamp for depth image
		 */
		public void process(Planar<GrayU8> rgb, GrayU16 depth, long timeRgb, long timeDepth);
	}
}
