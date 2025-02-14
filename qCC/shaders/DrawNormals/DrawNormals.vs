#version 330 core

layout(location = 0) in vec3 vertexIn;
layout(location = 1) in uint normalIn;

out Vertex
{
    vec3 normal;
} vertex;

void decompress(uint index, out vec3 normal);

void main(void)
{
    gl_Position = vec4(vertexIn, 1.0);
    vec3 out_normal = vec3(0.0);
    decompress(normalIn, vertex.normal);
}

// Decompress Normals
void decompress(uint index, out vec3 normal)
{
    //hardcoded maxlevel
    if (index == 2097152u)
    {
        normal = vec3(0.0);
        return;
    }
    const uint level = 9u;
    
    float box[6] = float[6](0.0, 0.0, 0.0, 1.0, 1.0, 1.0);
    bool flip = false;

    uint l_shift = 18u; // level *2
    for (uint k = 0u; k < level; ++k)
    {
        l_shift = l_shift - 2u;
        uint sector = ((index >> l_shift) & 0x3u);
        if (flip)
        {
            float tmp = box[sector];
            box[0] = (box[0] + box[3]) / 2.0;
            box[1] = (box[1] + box[4]) / 2.0;
            box[2] = (box[2] + box[5]) / 2.0;
            if (sector != 3u)
            {
                box[(3u + sector)] = box[sector];
                box[sector] = tmp;
            }
            else
            {
                flip = false;
            }
        }
        else
        {
            float tmp = (sector != 3u) ? box[3u + sector] : 0.0;
            box[3] = (box[0] + box[3]) / 2.0;
            box[4] = (box[1] + box[4]) / 2.0;
            box[5] = (box[2] + box[5]) / 2.0;
            if (sector != 3u)
            {
                box[sector] = box[(3u + sector)];
                box[(3u + sector)] = tmp;
            }
            else
            {
                flip = true;
            }
        }
    }

    uint sector = (index >> 18u);

    normal[0] = (((sector & 0x4u) != 0u) ? -(box[3] + box[0]) : box[3] + box[0]);
    normal[1] = (((sector & 0x2u) != 0u) ? -(box[4] + box[1]) : box[4] + box[1]);
    normal[2] = (((sector & 0x1u) != 0u) ? -(box[5] + box[2]) : box[5] + box[2]);
}
