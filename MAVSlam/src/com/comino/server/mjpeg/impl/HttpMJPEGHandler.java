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


package com.comino.server.mjpeg.impl;

import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.imageio.ImageIO;

import com.comino.msp.model.DataModel;
import com.comino.msp.utils.ExecutorService;
import com.comino.realsense.boofcv.RealSenseInfo;
import com.comino.server.mjpeg.IMJPEGOverlayListener;
import com.comino.server.mjpeg.IVisualStreamHandler;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;

import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;

public class HttpMJPEGHandler implements HttpHandler, IVisualStreamHandler  {

	private static final int MAX_VIDEO_RATE_MS = 40;

	private List<IMJPEGOverlayListener> listeners = null;
	private BufferedImage image = null;
	private DataModel model = null;

	private  List<BufferedImage>imageByteList;
	private Graphics gr;
	private long last_image_tms = 0;

	public HttpMJPEGHandler(RealSenseInfo info, DataModel model) {
		this.model = model;
		this.imageByteList = new ArrayList<BufferedImage>(0);
		this.listeners = new ArrayList<IMJPEGOverlayListener>();
		this.image = new BufferedImage(info.width, info.height, BufferedImage.TYPE_BYTE_GRAY);
		this.gr =  image.getGraphics();
	}

	@Override
	public void handle(HttpExchange he) throws IOException {
		he.getResponseHeaders().add("content-type","multipart/x-mixed-replace; boundary=--BoundaryString");
		he.sendResponseHeaders(200, 0);
		OutputStream os = he.getResponseBody();
		while(true) {
			if(imageByteList.size() > 0) {
				os.write(("--BoundaryString\r\nContent-type:image/jpeg content-length:1\r\n\r\n").getBytes());
				ImageIO.write(imageByteList.get(0), "jpg", os );
				os.write("\r\n\r\n".getBytes());
				imageByteList.remove(0);
			}
			try {
				TimeUnit.MILLISECONDS.sleep(20);
			} catch (InterruptedException e) {	}
		}
	}

	@Override
	public void registerOverlayListener(IMJPEGOverlayListener listener) {
		this.listeners.add(listener);
	}

	@Override
	public void addToStream(GrayU8 grayImage, GrayU16 depth, DataModel model, long tms_us) {

		if((System.currentTimeMillis()-last_image_tms)<MAX_VIDEO_RATE_MS)
			return;

		last_image_tms = System.currentTimeMillis();

		if(imageByteList.size()>10) {
			imageByteList.remove(0);
			return;
		}

		ExecutorService.get().execute(() -> {
			if(listeners.size()>0) {
				ConvertBufferedImage.convertTo(grayImage, image);
				for(IMJPEGOverlayListener listener : listeners)
					listener.processOverlay(gr);
			}
			imageByteList.add(image);
		});
	}
}
