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


package com.comino.server.mjpeg;

import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.io.IOException;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import javax.imageio.ImageIO;

import com.comino.msp.model.DataModel;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;

import boofcv.io.image.ConvertBufferedImage;
import boofcv.struct.image.GrayU8;

public class MJPEGHandler implements HttpHandler  {

	private static final int MAX_VIDEO_RATE_MS = 50;

	private List<IMJPEGOverlayListener> listeners = null;
	private BufferedImage image = null;
	private DataModel model = null;

	private  List<BufferedImage>imageByteList;
	private Graphics gr;
	private long last_image_tms = 0;

	public MJPEGHandler(DataModel model) {
		this.model = model;
		this.imageByteList = new ArrayList<BufferedImage>(0);
		this.listeners = new ArrayList<IMJPEGOverlayListener>();
		this.image = new BufferedImage(320, 240, BufferedImage.TYPE_BYTE_GRAY);
		this.gr =  image.getGraphics();

		this.registerOverlayListener(ctx -> {
			if(!Float.isNaN(model.sys.t_armed_ms))
				ctx.drawString("Time:"+(int)model.sys.t_armed_ms+"ms", 10, 20);
		});
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
				TimeUnit.MILLISECONDS.sleep(10);
			} catch (InterruptedException e) {	}
		}
	}

	public void registerOverlayListener(IMJPEGOverlayListener listener) {
		this.listeners.add(listener);
	}

	public void addImage(GrayU8 bands) {

		if((System.currentTimeMillis()-last_image_tms)<MAX_VIDEO_RATE_MS)
			return;
		last_image_tms = System.currentTimeMillis();

		if(imageByteList.size()>10)
			imageByteList.remove(0);

		ConvertBufferedImage.convertTo(bands, image);
		if(listeners.size()>0) {
			for(IMJPEGOverlayListener listener : listeners)
				listener.processOverlay(gr);
		}
		imageByteList.add(image);
	}
}
