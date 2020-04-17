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

uniform sampler2D tex;

uniform uniforms{
uniform int samplingMode;
uniform ivec2 sourceDimensions;
uniform ivec2 targetDimensions;
};
//#extension GL_GOOGLE_include_directive : enable
//#include "scale/bicubic.vert.glsl"
// Cubic filter with Catmull-Rom parameters
float catmull_rom(float x)
{
    /*
     * Generally favorable results in image upscaling are given by a cubic filter with the values b = 0 and c = 0.5.
     * This is known as the Catmull-Rom filter, and it closely approximates Jinc upscaling with Lanczos input values.
     * Placing these values into the piecewise equation gives us a more compact representation of:
     *  y = 1.5 * abs(x)^3 - 2.5 * abs(x)^2 + 1                 // abs(x) < 1
     *  y = -0.5 * abs(x)^3 + 2.5 * abs(x)^2 - 4 * abs(x) + 2   // 1 <= abs(x) < 2
     */

    float t = abs(x);
    float t2 = t * t;
    float t3 = t * t * t;

    if (t < 1)
        return 1.5 * t3 - 2.5 * t2 + 1.0;
    else if (t < 2)
        return -0.5 * t3 + 2.5 * t2 - 4.0 * t + 2.0;
    else
        return 0.0;
}

float mitchell(float x)
{
    /*
     * This is another cubic filter with less aggressive sharpening than Catmull-Rom, which some users may prefer.
     * B = 1/3, C = 1/3.
     */

    float t = abs(x);
    float t2 = t * t;
    float t3 = t * t * t;

    if (t < 1)
        return 7.0/6.0 * t3 + -2.0 * t2 + 8.0/9.0;
    else if (t < 2)
        return -7.0/18.0 * t3 + 2.0 * t2 - 10.0/3.0 * t + 16.0/9.0;
    else
        return 0.0;
}

#define CR_AR_STRENGTH 0.9

#define FLT_MAX 3.402823466e+38
#define FLT_MIN 1.175494351e-38

// Calculates the distance between two points
float d(vec2 pt1, vec2 pt2)
{
    vec2 v = pt2 - pt1;
    return sqrt(dot(v,v));
}

// Samples a texture using a 4x4 kernel.
vec4 textureCubic(sampler2D sampler, vec2 texCoords, int mode){
    vec2 texSize = textureSize(sampler, 0);
    vec2 texelSize = 1.0 / texSize;
    vec2 texelFCoords = texCoords * texSize;
    texelFCoords -= 0.5;

    vec4 nSum = vec4( 0.0, 0.0, 0.0, 0.0 );
    vec4 nDenom = vec4( 0.0, 0.0, 0.0, 0.0 );

    vec2 coordFract = fract(texelFCoords);
    texCoords -= coordFract * texelSize;

    vec4 c;

    if (mode == SAMPLING_CATROM)
    {
        // catrom benefits from anti-ringing, which requires knowledge of the minimum and maximum samples in the kernel
        vec4 min_sample = vec4(FLT_MAX);
        vec4 max_sample = vec4(FLT_MIN);
        for (int m = -1; m <= 2; m++)
        {
            for (int n = -1; n <= 2; n++)
            {
                // this would use texelFetch, but that would require manual implementation of texture wrapping
                vec4 vecData = texture(sampler, texCoords + vec2(m, n) * texelSize);

                // update min and max as we go
                min_sample = min(min_sample, vecData);
                max_sample = max(max_sample, vecData);

                // calculate weight based on distance of the current texel offset from the sub-texel position of the sampling location
                float w = catmull_rom( d(vec2(m, n), coordFract) );

                // build the weighted average
                nSum += vecData * w;
                nDenom += w;
            }
        }
        // calculate weighted average
        c = nSum / nDenom;

        // store value before anti-ringing
        vec4 aux = c;
        // anti-ringing: clamp the color value so that it cannot exceed values already present in the kernel area
        c = clamp(c, min_sample, max_sample);
        // mix according to anti-ringing strength
        c = mix(aux, c, CR_AR_STRENGTH);
    }
    else if (mode == SAMPLING_MITCHELL)
    {
        for (int m = -1; m <= 2; m++)
        {
            for (int n = -1; n <= 2; n++)
            {
                // this would use texelFetch, but that would require manual implementation of texture wrapping
                vec4 vecData = texture(sampler, texCoords + vec2(m, n) * texelSize);

                // calculate weight based on distance of the current texel offset from the sub-texel position of the sampling location
                float w = mitchell( d(vec2(m, n), coordFract) );

                // build the weighted average
                nSum += vecData * w;
                nDenom += w;
            }
        }
        // calculate weighted average
        c = nSum / nDenom;
    }

    // return the weighted average
    return c;
}

//#include "scale/xbr_lv2_frag.frag.glsl"
#define CORNER_C
//#define CORNER_D

#define XBR_Y_WEIGHT 50.0           // involved in preserving small details if small_details = 1, otherwise unused
#define XBR_EQ_THRESHOLD 9.0        // equality threshold for comparisons
//#define XBR_LV1_COEFFICIENT 0.5   // unused, probably left over from a previous iteration
#define XBR_LV2_COEFFICIENT 2.0     // moves the step in a step function at one point during blending
#define small_details 1.0           // 0 or 1, switches logic in a few spots to help preserve small details
// END PARAMETERS //

#define mul(a,b) (b*a)
#define lv2_cf XBR_LV2_COEFFICIENT
#ifndef CORNER_A
#define SMOOTH_TIPS
#endif

//const float coef         = 2.0; // unused
const vec3 rgbw          = vec3(14.352, 28.176, 5.472);     // rgb weights
//const vec4 eq_threshold  = vec4(15.0, 15.0, 15.0, 15.0); // unused

const  vec4 Ao = vec4( 1.0, -1.0, -1.0, 1.0 );
const  vec4 Bo = vec4( 1.0,  1.0, -1.0,-1.0 );
const  vec4 Co = vec4( 1.5,  0.5, -0.5, 0.5 );
const  vec4 Ax = vec4( 1.0, -1.0, -1.0, 1.0 );
const  vec4 Bx = vec4( 0.5,  2.0, -0.5,-2.0 );
const  vec4 Cx = vec4( 1.0,  1.0, -0.5, 0.0 );
const  vec4 Ay = vec4( 1.0, -1.0, -1.0, 1.0 );
const  vec4 By = vec4( 2.0,  0.5, -2.0,-0.5 );
const  vec4 Cy = vec4( 2.0,  0.0, -1.0, 0.5 );
const  vec4 Ci = vec4(0.25, 0.25, 0.25, 0.25);

const vec3 Y = vec3(0.2126, 0.7152, 0.0722); // rec.709 luma weights

// Difference between vector components.
vec4 df(vec4 A, vec4 B)
{
    return vec4(abs(A-B));
}

// Compare two vectors and return their components are different.
vec4 diff(vec4 A, vec4 B)
{
    return vec4(notEqual(A, B));
}

// Determine if two vector components are equal based on a threshold.
vec4 eq(vec4 A, vec4 B)
{
    return (step(df(A, B), vec4(XBR_EQ_THRESHOLD)));
}

// Determine if two vector components are NOT equal based on a threshold.
vec4 neq(vec4 A, vec4 B)
{
    return (vec4(1.0, 1.0, 1.0, 1.0) - eq(A, B));
}

// Weighted distance.
vec4 wd(vec4 a, vec4 b, vec4 c, vec4 d, vec4 e, vec4 f, vec4 g, vec4 h)
{
    return (df(a,b) + df(a,c) + df(d,e) + df(d,f) + 4.0*df(g,h));
}

vec4 weighted_distance(vec4 a, vec4 b, vec4 c, vec4 d, vec4 e, vec4 f, vec4 g, vec4 h, vec4 i, vec4 j, vec4 k, vec4 l)
{
    return (df(a,b) + df(a,c) + df(d,e) + df(d,f) + df(i,j) + df(k,l) + 2.0*df(g,h));
}

float c_df(vec3 c1, vec3 c2)
{
    vec3 df = abs(c1 - c2);
    return df.r + df.g + df.b;
}

//#extension GL_GOOGLE_include_directive : enable
//#include "scale/xbr_lv2_common.comp.glsl"
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


// xBR-level2 upscaler. Level 2 means it detects edges in 2 directions, instead of just 1 in the most basic form of the algorithm.
// This improves quality by a good bit without adding too much complexity compared to available level-3 and level-4 algorithms.
vec4 textureXBR(sampler2D image, vec2 texCoord, XBRTable t, float scale)
{
    vec4 delta   = vec4(1.0/scale, 1.0/scale, 1.0/scale, 1.0/scale);
    vec4 delta_l = vec4(0.5/scale, 1.0/scale, 0.5/scale, 1.0/scale);
    vec4 delta_u = delta_l.yxwz;

    vec2 textureDimensions = textureSize(image, 0);

    vec4 edri, edr, edr_l, edr_u, px; // px = pixel, edr = edge detection rule
    vec4 irlv0, irlv1, irlv2l, irlv2u, block_3d;
    vec4 fx, fx_l, fx_u; // inequations of straight lines.

    vec2 fp = fract(texCoord*textureDimensions);

    //    A1 B1 C1
    // A0  A  B  C C4
    // D0  D  E  F F4
    // G0  G  H  I I4
    //    G5 H5 I5
    vec4 A1 = texture(image, t.t1.xw );
    vec4 B1 = texture(image, t.t1.yw );
    vec4 C1 = texture(image, t.t1.zw );
    vec4 A  = texture(image, t.t2.xw );
    vec4 B  = texture(image, t.t2.yw );
    vec4 C  = texture(image, t.t2.zw );
    vec4 D  = texture(image, t.t3.xw );
    vec4 E  = texture(image, t.t3.yw );
    vec4 F  = texture(image, t.t3.zw );
    vec4 G  = texture(image, t.t4.xw );
    vec4 H  = texture(image, t.t4.yw );
    vec4 I  = texture(image, t.t4.zw );
    vec4 G5 = texture(image, t.t5.xw );
    vec4 H5 = texture(image, t.t5.yw );
    vec4 I5 = texture(image, t.t5.zw );
    vec4 A0 = texture(image, t.t6.xy );
    vec4 D0 = texture(image, t.t6.xz );
    vec4 G0 = texture(image, t.t6.xw );
    vec4 C4 = texture(image, t.t7.xy );
    vec4 F4 = texture(image, t.t7.xz );
    vec4 I4 = texture(image, t.t7.xw );

    vec4 b  = vec4(dot(B.xyz ,rgbw), dot(D.xyz ,rgbw), dot(H.xyz ,rgbw), dot(F.xyz ,rgbw));
    vec4 c  = vec4(dot(C.xyz ,rgbw), dot(A.xyz ,rgbw), dot(G.xyz ,rgbw), dot(I.xyz ,rgbw));
    vec4 d  = b.yzwx;
    vec4 e  = vec4(dot(E.xyz,rgbw));
    vec4 f  = b.wxyz;
    vec4 g  = c.zwxy;
    vec4 h  = b.zwxy;
    vec4 i  = c.wxyz;

    vec4 i4, i5, h5, f4;

    float y_weight = XBR_Y_WEIGHT;

    if (small_details < 0.5)
    {
        i4 = vec4(dot(I4.xyz,rgbw), dot(C1.xyz,rgbw), dot(A0.xyz,rgbw), dot(G5.xyz,rgbw));
        i5 = vec4(dot(I5.xyz,rgbw), dot(C4.xyz,rgbw), dot(A1.xyz,rgbw), dot(G0.xyz,rgbw));
        h5 = vec4(dot(H5.xyz,rgbw), dot(F4.xyz,rgbw), dot(B1.xyz,rgbw), dot(D0.xyz,rgbw));
    }
    else
    {
        i4 = mul( mat4x3(I4.xyz, C1.xyz, A0.xyz, G5.xyz), y_weight * Y );
        i5 = mul( mat4x3(I5.xyz, C4.xyz, A1.xyz, G0.xyz), y_weight * Y );
        h5 = mul( mat4x3(H5.xyz, F4.xyz, B1.xyz, D0.xyz), y_weight * Y );
    }

    // These inequations define the line below which interpolation occurs.
    fx   = (Ao*fp.y+Bo*fp.x);
    fx_l = (Ax*fp.y+Bx*fp.x);
    fx_u = (Ay*fp.y+By*fp.x);

    // corner detection
    irlv1 = irlv0 = diff(e,f) * diff(e,h);
    #ifdef CORNER_B
    irlv1      = (irlv0 * ( neq(f,b) * neq(h,d) + eq(e,i) * neq(f,i4) * neq(h,i5) + eq(e,g) + eq(e,c) ) );
    #endif
    #ifdef CORNER_D
    vec4 c1 = i4.yzwx;
    vec4 g0 = i5.wxyz;
    irlv1     = (irlv0  *  ( neq(f,b) * neq(h,d) + eq(e,i) * neq(f,i4) * neq(h,i5) + eq(e,g) + eq(e,c) ) * (diff(f,f4) * diff(f,i) + diff(h,h5) * diff(h,i) + diff(h,g) + diff(f,c) + eq(b,c1) * eq(d,g0)));
    #endif
    #ifdef CORNER_C
    irlv1     = (irlv0  * ( neq(f,b) * neq(f,c) + neq(h,d) * neq(h,g) + eq(e,i) * (neq(f,f4) * neq(f,i4) + neq(h,h5) * neq(h,i5)) + eq(e,g) + eq(e,c)) );
    #endif

    // corner detection in the other direction
    irlv2l = diff(e,g) * diff(d,g);
    irlv2u = diff(e,c) * diff(b,c);

    vec4 fx45i = clamp((fx   + delta   -Co - Ci)/(2.0*delta  ), 0.0, 1.0);
    vec4 fx45  = clamp((fx   + delta   -Co     )/(2.0*delta  ), 0.0, 1.0);
    vec4 fx30  = clamp((fx_l + delta_l -Cx     )/(2.0*delta_l), 0.0, 1.0);
    vec4 fx60  = clamp((fx_u + delta_u -Cy     )/(2.0*delta_u), 0.0, 1.0);

    vec4 wd1, wd2;
    if (small_details < 0.5)
    {
        wd1 = wd( e, c,  g, i, h5, f4, h, f);
        wd2 = wd( h, d, i5, f, i4,  b, e, i);
    }
    else
    {
        wd1 = weighted_distance( e, c, g, i, f4, h5, h, f, b, d, i4, i5);
        wd2 = weighted_distance( h, d, i5, f, b, i4, e, i, g, h5, c, f4);
    }

    edri  = step(wd1, wd2) * irlv0;
    edr   = step(wd1 + vec4(0.1, 0.1, 0.1, 0.1), wd2) * step(vec4(0.5, 0.5, 0.5, 0.5), irlv1);
    edr_l = step( lv2_cf*df(f,g), df(h,c) ) * irlv2l * edr;
    edr_u = step( lv2_cf*df(h,c), df(f,g) ) * irlv2u * edr;

    fx45  = edr   * fx45;
    fx30  = edr_l * fx30;
    fx60  = edr_u * fx60;
    fx45i = edri  * fx45i;

    px = step(df(e,f), df(e,h));

    #ifdef SMOOTH_TIPS
    vec4 maximos = max(max(fx30, fx60), max(fx45, fx45i));
    #endif
    #ifndef SMOOTH_TIPS
    vec4 maximos = max(max(fx30, fx60), fx45);
    #endif

    vec4 res1 = E;
    res1 = mix(res1, mix(H, F, px.x), maximos.x);
    res1 = mix(res1, mix(B, D, px.z), maximos.z);

    vec4 res2 = E;
    res2 = mix(res2, mix(F, B, px.y), maximos.y);
    res2 = mix(res2, mix(D, H, px.w), maximos.w);

    vec4 res = mix(res1, res2, step(c_df(E.xyz, res1.xyz), c_df(E.xyz, res2.xyz)));

    return res;
}


in vec2 TexCoord;
in XBRTable xbrTable;

out vec4 FragColor;

void main() {
    vec4 c;

    if (samplingMode == SAMPLING_DEFAULT)
        c = texture(tex, TexCoord);
    else if (samplingMode == SAMPLING_CATROM || samplingMode == SAMPLING_MITCHELL)
        c = textureCubic(tex, TexCoord, samplingMode);
    else if (samplingMode == SAMPLING_XBR)
        c = textureXBR(tex, TexCoord, xbrTable, ceil(1.0 * targetDimensions.x / sourceDimensions.x));

    FragColor = c;
}