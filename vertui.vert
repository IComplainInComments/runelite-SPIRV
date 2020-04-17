#version 430 core
/*
 * Copyright (c) 2018, Adam <Adam@sigterm.info>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define SAMPLING_DEFAULT 0
#define SAMPLING_MITCHELL 1
#define SAMPLING_CATROM 2
#define SAMPLING_XBR 3


uniform uniforms {
int samplingMode;
ivec2 sourceDimensions;
ivec2 targetDimensions;
};
//#extension GL_GOOGLE_include_directive : enable
//#include "scale/xbr_lv2_vert.vert.glsl"
struct XBRTable
{
    vec2 texCoord;
    vec4 t1;
    vec4 t2;
    vec4 t3;
    vec4 t4;
    vec4 t5;
    vec4 t6;
    vec4 t7;
};


XBRTable xbr_vert(vec2 texCoord, ivec2 sourceDimensions)
{
    float dx = (1.0/sourceDimensions.x);
    float dy = (1.0/sourceDimensions.y);

    // Define coordinates to optimize later fetching of adjacent pixels
    //    A1 B1 C1
    // A0  A  B  C C4
    // D0  D  E  F F4
    // G0  G  H  I I4
    //    G5 H5 I5
    XBRTable tab = XBRTable(
        texCoord,
        texCoord.xxxy + vec4( -dx, 0, dx,-2.0*dy), // A1 B1 C1
        texCoord.xxxy + vec4( -dx, 0, dx,    -dy), //  A  B  C
        texCoord.xxxy + vec4( -dx, 0, dx,      0), //  D  E  F
        texCoord.xxxy + vec4( -dx, 0, dx,     dy), //  G  H  I
        texCoord.xxxy + vec4( -dx, 0, dx, 2.0*dy), // G5 H5 I5
        texCoord.xyyy + vec4(-2.0*dx,-dy, 0,  dy), // A0 D0 G0
        texCoord.xyyy + vec4( 2.0*dx,-dy, 0,  dy) // C4 F4 I4
    );

    tab.texCoord.x *= 1.00000001;

    return tab;
}

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord;

out vec2 TexCoord;
out XBRTable xbrTable;

void main()
{
    gl_Position = vec4(aPos, 1.0);
    TexCoord = aTexCoord;

    if (samplingMode == SAMPLING_XBR)
        xbrTable = xbr_vert(TexCoord, sourceDimensions);
}