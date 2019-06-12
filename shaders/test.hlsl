
Buffer<float4> g_buffer : register(t0);

cbuffer ConstantBuffer : register(b0)
{
	matrix worldViewProj;
	matrix world;
	float4 lightDir;
	float4 eye;
	//float ambientLight;
	//float diffuseLight;
}

struct VertexShaderOutput
{
	float4 position : SV_POSITION;
	float4 normal : NORMAL0;
	float4 color : TEXCOORD0;
};

VertexShaderOutput vertexShaderMain(float4 position : position,
									float4 normal : normal,
									float4 color : color,
									float4 texCoord : texCoord,
									uint vID : SV_VertexID)
{
	VertexShaderOutput output;
	output.position = mul(position, worldViewProj);
	output.color = color;
	output.normal = normal;

	float4 bufferValue = g_buffer[vID];
	output.color = bufferValue;

	return output;
}


[maxvertexcount(3)]
void geometryShaderMain(triangle VertexShaderOutput tri[3], inout TriangleStream<VertexShaderOutput> triStream)
{
	VertexShaderOutput output;

	[unroll]
	for (uint c = 0; c < 3; c++)
	{
		output = tri[c];
		triStream.Append(output);
	}
}

float4 pixelShaderMain(VertexShaderOutput input) : SV_Target
{
	//return float4(input.color.x, input.color.y, input.color.z, 1.0f);

	// Phong relfection is ambient + light-diffuse + spec highlights.
	// Get light direction for this fragment
	float3 light_dir = normalize(lightDir.xyz - input.position);

	float diffuse = 0.4;
	float ambient = 0.6;
	
	float diffuse_lighting = diffuse * (dot(input.normal, -light_dir)); // per pixel diffuse lighting
	float ambient_lighting = ambient;

	float lighting = diffuse_lighting + ambient_lighting;
	return float4(input.color.x * lighting, input.color.y * lighting, input.color.z * lighting, 0.9f);	
}
