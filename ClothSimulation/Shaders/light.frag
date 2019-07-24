#version 330 core
in vec3 fragNormal;
in vec4 shadowCoord;
uniform vec3 matColor;
out vec4 fragColor;
uniform sampler2D shadowMap;

void main()
{   
    vec3 projCoords = shadowCoord.xyz / shadowCoord.w;
    projCoords = projCoords * 0.5 + 0.5;
    vec3 n = normalize(fragNormal);
    if(!gl_FrontFacing){
        n *= -1.0;
    }
    vec3 lightDir = vec3(0, 0.707106781, 0.707106781);
    float diff = max(dot(n, lightDir), 0.0);
    float amb = 0.2;
    float closestDepth = texture(shadowMap, projCoords.xy).r; 
    float fragDepth = projCoords.z;
    float bias = max(0.000005 * (1.0 - dot(n, lightDir)), 0.0000005);
    vec2 texelSize = 1.0 / textureSize(shadowMap, 0);
    float visibility = 0.0;
    for(int x = -1; x <= 1; ++x){
        for(int y = -1; y <= 1; ++y){
            float pcfDepth = texture(shadowMap, projCoords.xy + vec2(x, y) * texelSize).r; 
            visibility += fragDepth - bias > pcfDepth  ? 1.0 : 0.0;        
        }    
    }
    visibility /= 9.0;
    fragColor = vec4(matColor.r*(diff*(1.0 - visibility) + amb), matColor.g*(diff*(1.0 - visibility) + amb), matColor.b*(diff*(1.0 - visibility) + amb), 1.0f);     
}