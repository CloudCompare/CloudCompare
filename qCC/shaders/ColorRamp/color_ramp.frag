// Color Ramp Shader (CloudCompare - 04/18/2013)

uniform float minSaturation;		//minimum saturation value (between 0 and 1)
uniform float maxSaturation;		//maximum saturation value (between 0 and 1)

uniform float colormap[256];		//float-packed RGB colors (max: 256)
uniform int colormapSize;			//colormap size
uniform float colorGray;			//color for grayed-out points

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
		int rampPosi = 0;
		if (gl_Color[0] <= minSaturation)
			rampPosi = 0;
		else if (gl_Color[0] >= maxSaturation)
			rampPosi = 255;
		else
			rampPosi = int((gl_Color[0]-minSaturation)/(maxSaturation-minSaturation)*float(colormapSize));
		
		//unpack the corresponding color
		unpackedValues = fract(unpackedValues * colormap[rampPosi]);
	}
	else //grayed point
	{
		unpackedValues = fract(unpackedValues * colorGray);
	}

	//modulate unpacked color with true lighting value
	gl_FragColor = vec4( unpackedValues * gl_Color[2], gl_Color[3]);
}
