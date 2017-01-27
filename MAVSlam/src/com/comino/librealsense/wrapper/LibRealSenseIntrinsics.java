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

package com.comino.librealsense.wrapper;

import com.comino.librealsense.wrapper.LibRealSenseWrapper.rs_intrinsics;

import boofcv.struct.calib.IntrinsicParameters;

public class LibRealSenseIntrinsics extends IntrinsicParameters {

	private static final long serialVersionUID = -3525224116201930353L;

	public int model=0;

	public LibRealSenseIntrinsics(rs_intrinsics intrinsics) {

	   this.model = intrinsics.model;

       this.cx = intrinsics.ppx;
       this.cy = intrinsics.ppy;

       this.width  = intrinsics.width;
       this.height = intrinsics.height;

       this.fx = intrinsics.fx;
       this.fy = intrinsics.fy;

       this.radial = new double[5];
       for(int i=0;i<intrinsics.coeffs.length;i++)
    	   this.radial[i] = intrinsics.coeffs[i];

       this.t1 = 0;
       this.t2 = 0;



	}

	public String toString() {
		return "cx="+cx+" cy="+cy+" fx="+fx+" fy="+fy;
	}

}
