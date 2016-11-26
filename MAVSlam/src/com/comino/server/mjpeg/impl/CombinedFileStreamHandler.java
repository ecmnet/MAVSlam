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

package com.comino.server.mjpeg.impl;

import java.io.Serializable;
import java.util.HashMap;
import java.util.Map;

import com.comino.msp.model.DataModel;
import com.comino.realsense.boofcv.RealSenseInfo;
import com.comino.server.mjpeg.IMJPEGOverlayListener;
import com.comino.server.mjpeg.IVisualStreamHandler;

import boofcv.struct.image.GrayU16;
import boofcv.struct.image.GrayU8;

public class CombinedFileStreamHandler implements IVisualStreamHandler, Runnable {

	private Map<Long,StreamDataSet> dataSetStream = null;


	public CombinedFileStreamHandler(RealSenseInfo info) {
		dataSetStream = new HashMap<Long,StreamDataSet>();
	}

	@Override
	public void addToStream(GrayU8 grayImage, GrayU16 depth, DataModel model, long tms_us) {
		long t = System.currentTimeMillis();
		dataSetStream.put(tms_us,new StreamDataSet(grayImage,depth,model,tms_us));
		System.out.println(dataSetStream.size()+": "+tms_us+" - "+(System.currentTimeMillis()-t));

	}

	@Override
	public void registerOverlayListener(IMJPEGOverlayListener listener) {

	}


	private class StreamDataSet implements Serializable {

		private static final long serialVersionUID = -4516830726585551719L;

		private GrayU8 		grayImage;
		private GrayU16 	depthImage;
		private DataModel 	model;
		private  long 		tms_us;

		public StreamDataSet(GrayU8 g, GrayU16 d, DataModel m, long t) {
			this.grayImage  = g.clone();
			this.depthImage = d.clone();
			this.model      = m.clone();
			this.tms_us     = t;
		}

	}


	@Override
	public void run() {
		// TODO Auto-generated method stub

	}

}
