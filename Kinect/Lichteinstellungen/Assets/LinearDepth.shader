Shader "Custom/LinearDepth"
{
    SubShader
    {
        Tags { "RenderType"="Opaque" }
        Pass
        {
            ZWrite Off
            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            struct appdata
            {
                float4 vertex : POSITION;
            };

            struct v2f
            {
                float4 pos : SV_POSITION;
                float depth : TEXCOORD0;
            };

            v2f vert (appdata v)
            {
                v2f o;
                o.pos = UnityObjectToClipPos(v.vertex);
                o.depth = o.pos.z / o.pos.w; // Non-linear depth
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                float linearDepth = Linear01Depth(i.depth); // Linearisiert zwischen near und far
                return fixed4(linearDepth, 0, 0, 1); // Tiefenwert im Rotkanal
            }
            ENDCG
        }
    }
    FallBack Off
}

