#version 110

// Color Ramp Shader (CloudCompare - 04/23/2013)

uniform float uf_minSaturation;			//minimum saturation value (between 0 and 1)
uniform float uf_maxSaturation;			//maximum saturation value (between 0 and 1)

uniform float uf_colormapTable[256];	//float-packed RGB colors (max: 256)
uniform float uf_colormapSize;			//colormap size (as a float as we only use it as a float!)
uniform float uf_colorGray;				//color for grayed-out points

void main(void)
{
	//input: gl_Color
	// - gl_Color[0] = normalized scalar value
	// - gl_Color[1] = flag: whether point should be grayed (< 1.0) or not (1.0)
	// - gl_Color[2] = true lighting value
	//output: gl_FragColor
	
	vec3 unpackedValues = vec3(1.0, 256.0, 65536.0);
	
	if (gl_Color[1] > 0.99) //0.99 to cope with round-off issues (in perspective mode for instance)
	{
		//determine position in current colormap
		int rampPosi;
		if (gl_Color[0] <= uf_minSaturation)
			rampPosi = 0;
		else if (gl_Color[0] < uf_maxSaturation)
			rampPosi = int((gl_Color[0]-uf_minSaturation)*uf_colormapSize/(uf_maxSaturation-uf_minSaturation));
		else
			rampPosi = int(uf_colormapSize)-1;
		
		//unpack the corresponding color
		unpackedValues = fract(unpackedValues * uf_colormapTable[rampPosi]);
	}
	else //grayed point
	{
		unpackedValues = fract(unpackedValues * uf_colorGray);
	}

	//modulate unpacked color with true lighting value
	gl_FragColor = vec4(gl_Color[2] * unpackedValues, gl_Color[3]);
}
